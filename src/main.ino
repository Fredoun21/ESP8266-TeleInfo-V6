/*
Description du programme main.ino

*/
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <SoftwareSerial.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "Config.h"
#include "DRV_teleinfo.h"

// variable gestion de boucle
const int watchdog = 30000;              // Fréquence d'envoi des données à Domoticz 30s
unsigned long previousMillis = millis(); // mémoire pour envoi des données
boolean firstLoop = LOW;                 // Affichage au démarrage

// variable Téléinfo
char HHPHC;                 // Groupe horaire si option = heures creuses ou tempo
int ISOUSC;                 // intensité souscrite
int IINST;                  // intensité instantanée en A
int IMAX;                   // intensité maxi en A
int PAPP;                   // puissance apparente en VA
int PREAL;                  // puissance reelle en W
unsigned long HCHC;         // compteur Heures Creuses en W
unsigned long HCHP;         // compteur Heures Pleines en W
unsigned long PAPP_arrondi; // PAPP*497/500/16 arrondi
String PTEC;                // Régime actuel : HPJB, HCJB, HPJW, HCJW, HPJR, HCJR
String ADCO;                // Identifiant du compteur
String OPTARIF;             // option tarifaire (type d’abonnement)
String MOTDETAT;            // Mot d’état (autocontrôle)

/*
CREATION DES OBJETS
*/

// client MQTT
WiFiClient espClient;
PubSubClient clientMQTT(espClient);

// objet liaison série
SoftwareSerial mySerial;

OneWire oneWire(PIN_DS18B20);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

/*
setup
*/
void setup()
{
    Serial.begin(115200);

    pinMode(PIN_RELAY, OUTPUT);

    // Start LE DS18b20
    sensors.begin();

    // Connexion au réseau wifi
    setup_wifi(local_IP, gateway, subnet, ssid, password);
    delay(500);

    //Configuration de la connexion au serveur MQTT
    clientMQTT.setServer(MQTT_SERVER, 1883);
    //La fonction de callback qui est executée à chaque réception de message
    clientMQTT.setCallback(callback);

    // Configure la liaison série pour TéléInfo
    setupTeleInfo(SS_BAUDRATE, PIN_TELEINFO, SS_BUFFER_SIZE);
}

/*
loop
*/
void loop()
{
    // Connexion client MQTT
    if (!clientMQTT.connected())
    {
        reconnect(MQTT_ID, topic_Domoticz_OUT);
    }
    clientMQTT.loop();

    unsigned long currentMillis = millis();
    boolean teleInfoReceived;
    String cmd = "";

    // ajout d'un delais de 60s apres chaque trame envoyés pour éviter d'envoyer
    // en permanence des informations à domoticz et de créer des interférences
    if ((currentMillis - previousMillis > watchdog) || firstLoop == LOW)
    {
        previousMillis = currentMillis;
        // Mise à jour état relais
        if (firstLoop == LOW)
        {
            sendMqttToDomoticz(IDXRELAIS, String(digitalRead(PIN_RELAY)), topic_Domoticz_IN);
            // sendToDomoticz_Relais(IDXRELAIS, digitalRead(PIN_RELAY), topic_Domoticz_IN);
            firstLoop = HIGH;
        }

        // Envoi MQTT température du DS18B20
        sendMqttToDomoticz(IDXTEMP, String(retourSensor()), topic_Domoticz_IN);

        teleInfoReceived = readTeleInfo(&mySerial); // réception caractères de téléinfo
        if (teleInfoReceived)
        {
            displayTeleInfo(); // console pour voir les trames téléinfo
            if (0 < HCHC && 0 < HCHP)
            {
                //Envoi MQTT consommation elec
                cmd = String(HCHC) + ";" + String(HCHP) + ";0;0;" + String(PREAL) + ";0";
                sendMqttToDomoticz(IDXTELEINFO, cmd, topic_Domoticz_IN);
            }
            // Envoi MQTT courant instantané
            sendMqttToDomoticz(IDXIINST, String(IINST), topic_Domoticz_IN);
        }
    }
}

/*
setup_wifi
Connexion du module ESP au wifi
local_ip -> adressi IP du module
gateway -> passerelle réseau
subnet -> masque de sous réseau
ssid -> nom du SSID pour la connexion Wifi
password -> mot de passe pour la connexion Wifi 
 */
void setup_wifi(IPAddress local_ip, IPAddress gateway, IPAddress subnet, const char *ssid, const char *password)
{
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.println(F("Connection Wifi..."));
    Serial.println(ssid);

    WiFi.config(local_ip, gateway, subnet);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

/*
setupTeleInfo
Configure la liaison série pour TéléInfo
baud -> vitesse liaison série TéléInfo (1200bauds)
rxPin -> numéro broche pour la ligne Rx
buffer_size -> taille du buffer de la liaison série
*/
void setupTeleInfo(uint32_t baud, int8_t rxPin, int buffer_size)
{
    // Démarrage liaison série pour téléinfo
    Serial.println("Start SoftwareSerial");
    mySerial.begin(baud, SWSERIAL_7E1, rxPin, -1, false, buffer_size);
    mySerial.enableRx(true);  // Enable interrupts on the rx pin
    mySerial.enableTx(false); // One wire control.
    Serial.println("\nSoftware serial test started");
    delay(500);

    // Initialise les variable téléinfo
    ADCO = "270622224349";
    OPTARIF = "----";
    ISOUSC = 0;
    HCHC = 0L;     // compteur Heures Creuses en W
    HCHP = 0L;     // compteur Heures Pleines en W
    PTEC = "----"; // Régime actuel : HPJB, HCJB, HPJW, HCJW, HPJR, HCJR
    HHPHC = '-';
    IINST = 0; // intensité instantanée en A
    IMAX = 0;  // intensité maxi en A
    PAPP = 0;  // puissance apparente en VA
    MOTDETAT = "------";
}

/*
reconnect
Connexion server MQTT
id -> nom du client 
topic -> nom du topic pour envoyer les messages (domoticz/in)
*/
void reconnect(const char *id, const char *topic)
{
    //Boucle jusqu'à obtenur une reconnexion
    while (!clientMQTT.connected())
    {
        Serial.print("Connexion au serveur MQTT... Status= ");
        if (clientMQTT.connect(id, MQTT_USER, MQTT_PASSWORD))
        {
            Serial.println("OK");
            // suscribe to MQTT topics
            Serial.print("Subscribe to domoticz/out topic. Status= ");
            if (clientMQTT.subscribe(topic, 0))
                Serial.println("OK");
            else
            {
                Serial.print("KO, erreur: ");
                Serial.println(clientMQTT.state());
            };
        }
        else
        {
            Serial.print("KO, erreur : ");
            Serial.println(clientMQTT.state());
            Serial.println(" On attend 5 secondes avant de recommencer");
            delay(5000);
        }
    }
}

/*
callback
Déclenche les actions à la réception d'un message
topic -> nom du topic de réception des message (domoticz/out)
payload -> message reçu
length -> longueur message reçu
 */
void callback(char *topic, byte *payload, unsigned int length)
{
    DynamicJsonDocument jsonBuffer(MQTT_MAX_PACKET_SIZE);
    String messageReceived = "";

    // Affiche le topic entrant - display incoming Topic
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");

    // decode payload message
    for (int i = 0; i < length; i++)
    {
        messageReceived += ((char)payload[i]);
    }
    // display incoming message
    Serial.println("Message recu:");
    Serial.print(messageReceived);

    // if domoticz message
    if (strcmp(topic, topic_Domoticz_OUT) == 0)
    {
        DeserializationError error = deserializeJson(jsonBuffer, messageReceived);
        if (error)
        {
            Serial.print(F("parsing Domoticz/out JSON Received Message failed with code: "));
            Serial.println(error.c_str());
            return;
        }
        int idx = jsonBuffer["idx"];

#ifdef DEBUG
        Serial.print("IDX: ");
        Serial.println(idx);
#endif

        int cmde = jsonBuffer["nvalue"];

        if (idx == IDXRELAIS)
        {
            if (cmde == 0)
            {
                digitalWrite(PIN_RELAY, LOW);
            }
            else if (cmde == 1)
            {
                digitalWrite(PIN_RELAY, HIGH);
            }
            else
            {
                Serial.println("Erreur dans le message de commande du relais ");
            }
            Serial.print("\nLe relais est ");
            Serial.print(digitalRead(PIN_RELAY));
        }
    }
}

/*
sendMqttToDomoticz
Mise à jour valeur domoticz par messages MQTT 
idx -> adresse IDX domoticz du materiel
svalue -> donnée converti en String à envoyer 
topic -> topic pour envoyer les message(domoticz/in)
*/
void sendMqttToDomoticz(int idx, String svalue, const char *topic)
{
    char msgToPublish[MQTT_MAX_PACKET_SIZE];

    StaticJsonDocument<1024> doc;
    doc["idx"] = idx;
    doc["nvalue"] = 0;
    doc["svalue"] = svalue;
    serializeJson(doc, msgToPublish);
    Serial.print(msgToPublish);
    Serial.print(" Published to ");
    Serial.print(topic);
    Serial.print(". Status=");
    if (clientMQTT.publish(topic, msgToPublish))
        Serial.println("OK");
    else
        Serial.println("KO");
}

/*
readTeleInfo
Capture des trames de Teleinfo
ss -> nom de la liaison série loiciel
*/
boolean readTeleInfo(SoftwareSerial *ss)
{
#define startFrame 0x02 // caractère de début de trame "Start TeXt" STX (002h)
#define endFrame 0x03   // caractère de fin de trame "End TeXt » ETX (003h)
#define startLine 0x0A  // caratère de début de groupe d'information "Line Feed" LF (00Ah)
#define endLine 0x0D    // caratère fin de groupe d'informations "Carriage Return" CR (00Dh)
#define maxFrameLen 280 // longueur max d'une trame

#ifdef DEBUG
    Serial.println(2);
#endif

    int comptChar = 0; // variable de comptage des caractères reçus
    char charIn = 0;   // variable de mémorisation du caractère courant en réception

    char bufferTeleinfo[21] = "";
    int bufferLen = 0;
    int checkSum;

    int sequenceNumber = 0; // number of information group

    //--- wait for starting frame character

    while (charIn != startFrame)
    {
        // "Start Text" STX (002 h) is the beginning of the frame
        // Serial.printf("20 CharIn: %#x \n", charIn);
        if (ss->available())
        {
            charIn = ss->read() & 0x7F; // Serial.read() vide buffer au fur et à mesure
            // Serial.printf("21 CharIn: %#x \n", charIn);
        }
    }
    // fin while (tant que) pas caractère 0x02
    //  while (charIn != endFrame and comptChar<=maxFrameLen)
    while (charIn != endFrame)
    {
#ifdef DEBUG
        Serial.println(12);
#endif
        // tant que des octets sont disponibles en lecture : on lit les caractères
        if (ss->available())
        {
#ifdef DEBUG
            Serial.println(13);
#endif
            charIn = ss->read() & 0x7F;
#ifdef DEBUG
            Serial.println(charIn);
#endif
            // incrémente le compteur de caractère reçus
            comptChar++;
            if (charIn == startLine)
            {
#ifdef DEBUG
                Serial.println(14);
#endif
                bufferLen = 0;
            }
            bufferTeleinfo[bufferLen] = charIn;
            // on utilise une limite max pour éviter String trop long en cas erreur réception
            // ajoute le caractère reçu au String pour les N premiers caractères
            if (charIn == endLine)
            {
#ifdef DEBUG
                Serial.println(15);
#endif
                checkSum = bufferTeleinfo[bufferLen - 1];
                if (chksum(bufferTeleinfo, bufferLen) == checkSum)
                {
                    // we clear the 1st character
#ifdef DEBUG
                    Serial.println(16);
#endif
                    strncpy(&bufferTeleinfo[0], &bufferTeleinfo[1], bufferLen - 3);
                    bufferTeleinfo[bufferLen - 3] = 0x00;
                    sequenceNumber++;
                    if (!handleBuffer(bufferTeleinfo, sequenceNumber))
                    {
                        Serial.println(F("Sequence error ..."));
                        return false;
                    }
                }
                else
                {
                    Serial.println(F("Checksum error ..."));
                    return false;
                }
            }
            else
            {
                bufferLen++;
#ifdef DEBUG
                Serial.printf("17 - Buffer: %i", bufferLen);
#endif
            }
        }
        if (comptChar > maxFrameLen)
        {
            Serial.println(F("Overflow error ..."));
            return false;
        }
    }
    ss->flush();
    return true;
}

/*
handleBuffer
Décodage trame TéléInfo
bufferTeleinfo -> 
sequenceNumber -> 
*/
boolean handleBuffer(char *bufferTeleinfo, int sequenceNumber)
{
    char *resultString = strchr(bufferTeleinfo, ' ') + 1; // Crée un pointeur sur le premier caractère après l'espace
    boolean sequenceIsOK;

#ifdef DEBUG
    Serial.println(3);
#endif

    switch (sequenceNumber)
    {
    case 1:
        if (sequenceIsOK = bufferTeleinfo[0] == 'A')
            ADCO = String(resultString);
        break;
    case 2:
        if (sequenceIsOK = bufferTeleinfo[0] == 'O')
            OPTARIF = String(resultString);
        break;
    case 3:
        if (sequenceIsOK = bufferTeleinfo[1] == 'S')
            ISOUSC = atol(resultString);
        break;
    case 4:
        if (sequenceIsOK = bufferTeleinfo[3] == 'C')
            HCHC = atol(resultString);
        break;
    case 5:
        if (sequenceIsOK = bufferTeleinfo[3] == 'P')
            HCHP = atol(resultString);
        break;
    case 6:
        if (sequenceIsOK = bufferTeleinfo[1] == 'T')
            PTEC = String(resultString);
        break;
    case 7:
        if (sequenceIsOK = bufferTeleinfo[1] == 'I')
            IINST = atol(resultString);
        break;
    case 8:
        if (sequenceIsOK = bufferTeleinfo[1] == 'M')
            IMAX = atol(resultString);
        break;
    case 9:
        if (sequenceIsOK = bufferTeleinfo[1] == 'A')
            PAPP = atol(resultString);
        PREAL = PAPP * 0.98;
        break;
    case 10:
        if (sequenceIsOK = bufferTeleinfo[1] == 'H')
            HHPHC = resultString[0];
        break;
    case 11:
        if (sequenceIsOK = bufferTeleinfo[1] == 'O')
            MOTDETAT = String(resultString);
        break;
    }
#ifdef DEBUG
    if (!sequenceIsOK)
    {
        Serial.print(F("Out of sequence ..."));
        Serial.println(bufferTeleinfo);
    }
#endif
    return sequenceIsOK;
}

/*
chksum
Calcul de checksum réception TeleInfo
buff -> 
len -> 
*/
char chksum(char *buff, uint8_t len)
{
    int i;
    char sum = 0;
    for (i = 1; i < (len - 2); i++)
        sum = sum + buff[i];
    sum = (sum & 0x3F) + 0x20;
    return (sum);
}

/*
displayTeleInfo
Affiche sur la console le décodage d'une trame TeleInfo
*/
void displayTeleInfo()
{
    /*
    ADCO 270622224349 B
    OPTARIF HC.. <
    ISOUSC 30 9
    HCHC 014460852 $
    HCHP 012506372 -
    PTEC HP..
    IINST 002 Y
    IMAX 035 G
    PAPP 00520 (
    HHPHC C .
    MOTDETAT 000000 B
  */

    Serial.print(F(" "));
    Serial.println();
    Serial.print(F("ADCO "));
    Serial.println(ADCO);
    Serial.print(F("OPTARIF "));
    Serial.println(OPTARIF);
    Serial.print(F("ISOUSC "));
    Serial.println(ISOUSC);
    Serial.print(F("HCHC "));
    Serial.println(HCHC);
    Serial.print(F("HCHP "));
    Serial.println(HCHP);
    Serial.print(F("PTEC "));
    Serial.println(PTEC);
    Serial.print(F("IINST "));
    Serial.println(IINST);
    Serial.print(F("IMAX "));
    Serial.println(IMAX);
    Serial.print(F("PAPP "));
    Serial.println(PAPP);
    Serial.print(F("PREAL "));
    Serial.println(PREAL);
    Serial.print(F("HHPHC "));
    Serial.println(HHPHC);
    Serial.print(F("MOTDETAT "));
    Serial.println(MOTDETAT);
}

/*
retourSensor
Renvoie la température du DS18B20 en float
*/
float retourSensor()
{
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    // Serial.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    // After we got the temperatures, we can print them here.
    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    float tempC = sensors.getTempCByIndex(0);

    // Check if reading was successful
    if (tempC != DEVICE_DISCONNECTED_C)
    {
        Serial.print("Temperature mesurée: ");
        Serial.println(tempC);
    }
    else
    {
        Serial.println("Error: Pas de données de temperature disponible");
    }
    return tempC;
}