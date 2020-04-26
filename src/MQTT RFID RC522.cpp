/*
  Projet d'apprentissage d'un objet connecté (IoT)  pour réaliser une sonde de température
  ESP8266 + DS12B20 + LED + MQTT + Home-Assistant
  Projets DIY (https://www.projetsdiy.fr) - Mai 2016
  Licence : MIT
*/

// Include the libraries we need
#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <MFRC522.h>

#include "Config.h"

/*
 PIN SETTINGS
 */
#define PIN_RELAIS 16
// RFID pin
#define RST_PIN 5 // Configurable, see typical pin layout above
#define SS_PIN 4  // Configurable, see typical pin layout above

/*
CONFIGURATION RESEAU WIFI
*/
IPAddress local_IP(192, 168, 1, 95); //Adresse IP du module

/*
CONFIGURATION MQTT
*/
#define MQTT_ID "ESP52Client"

/*
CONFIGURATION DOMOTICZ
*/
#define IDXRFID 450        //IDX de ESPTest52 T°
#define IDXESP52RELAIS 451 //IDX de ESPTest52 Relais

/*
CREATION DES OBJETS
*/
// client MQTT
WiFiClient espClient;
PubSubClient clientMQTT(espClient);

MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
MFRC522::MIFARE_Key key;
byte nuidPICC[4]; // Init array that will store new NUID

void setup(void)
{
    Serial.begin(115200);

    pinMode(PIN_RELAIS, OUTPUT);

    setupRfid();

    rfidCheckFirmware();

    // Connexion au réseau wifi
    setup_wifi(local_IP, gateway, subnet, SSID, PASSWORD);
    delay(500);

#ifdef MQTT
    //Configuration de la connexion au serveur MQTT
    clientMQTT.setServer(MQTT_SERVER, 1883);
    //La fonction de callback qui est executée à chaque réception de message
    clientMQTT.setCallback(callback);
#endif
}

/*
 * Main function, get and show the temperature
 */
void loop(void)
{
#ifdef MQTT
    // Connexion client MQTT
    if (!clientMQTT.connected())
    {
        reconnect(MQTT_ID, TOPIC_DOMOTICZ_OUT);
    }
    clientMQTT.loop();
#endif

    rfidCheck();

    unsigned long currentMillis = millis();
    String cmd = "";

    // ajout d'un delais de 60s apres chaque trame envoyés pour éviter d'envoyer
    // en permanence des informations à domoticz et de créer des interférences
    if ((currentMillis - previousMillis > watchdog) || firstLoop == LOW)
    {
        previousMillis = currentMillis;
        firstLoop = HIGH;

        // Envoi MQTT température du DS18B20
        // sendMqttToDomoticz(IDXTEMP, String(retourSensor()), TOPIC_DOMOTICZ_IN);

        // Demande état d'un device
        // askMqttToDomoticz(IDXESP52RELAIS, "getdeviceinfo", TOPIC_DOMOTICZ_IN);
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
setupRfid
*/
void setupRfid()
{
    SPI.begin();        // Init SPI bus
    mfrc522.PCD_Init(); // Init MFRC522 module

    // Prepare the key (used both as key A and as key B)
    // using FFFFFFFFFFFFh which is the default at chip delivery from the factory
    for (byte i = 0; i < 6; i++)
    {
        key.keyByte[i] = 0xFF;
    }

    Serial.println(F("Scan a MIFARE Classic PICC to demonstrate read and write."));
    Serial.print(F("Using key (for A and B):"));
    dump_byte_array(key.keyByte, MFRC522::MF_KEY_SIZE);
    Serial.println();

    Serial.println(F("BEWARE: Data will be written to the PICC, in sector #1"));
}

/*
rfidCheck
*/
void rfidCheck()
{
    // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
    if (!mfrc522.PICC_IsNewCardPresent())
        return;

    // Select one of the cards
    if (!mfrc522.PICC_ReadCardSerial())
        return;

    // Show some details of the PICC (that is: the tag/card)
    Serial.print(F("Card UID:"));
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println();
    Serial.print(F("PICC type: "));
    MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    Serial.println(mfrc522.PICC_GetTypeName(piccType));

    // Check for compatibility
    if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI && piccType != MFRC522::PICC_TYPE_MIFARE_1K && piccType != MFRC522::PICC_TYPE_MIFARE_4K)
    {
        Serial.println(F("This sample only works with MIFARE Classic cards."));
        return;
    }

    // In this sample we use the second sector,
    // that is: sector #1, covering block #4 up to and including block #7
    byte sector = 1;
    byte blockAddr = 4;
    byte dataBlock[] = {
        0x01, 0x02, 0x03, 0x04, //  1,  2,   3,  4,
        0x05, 0x06, 0x07, 0x08, //  5,  6,   7,  8,
        0x09, 0x0a, 0xff, 0x0b, //  9, 10, 255, 11,
        0x0c, 0x0d, 0x0e, 0x0f  // 12, 13, 14, 15
    };
    byte trailerBlock = 7;
    MFRC522::StatusCode status;
    byte buffer[18];
    byte size = sizeof(buffer);

    // Authenticate using key A
    Serial.println(F("Authenticating using key A..."));
    status = (MFRC522::StatusCode)mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, trailerBlock, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK)
    {
        Serial.print(F("PCD_Authenticate() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return;
    }

    // Show the whole sector as it currently is
    Serial.println(F("Current data in sector:"));
    mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector);
    Serial.println();

    // Read data from the block
    Serial.print(F("Reading data from block "));
    Serial.print(blockAddr);
    Serial.println(F(" ..."));
    status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK)
    {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    Serial.print(F("Data in block "));
    Serial.print(blockAddr);
    Serial.println(F(":"));
    dump_byte_array(buffer, 16);
    Serial.println();
    Serial.println();

    // Authenticate using key B
    Serial.println(F("Authenticating again using key B..."));
    status = (MFRC522::StatusCode)mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_B, trailerBlock, &key, &(mfrc522.uid));
    if (status != MFRC522::STATUS_OK)
    {
        Serial.print(F("PCD_Authenticate() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
        return;
    }

    // Write data to the block
    Serial.print(F("Writing data into block "));
    Serial.print(blockAddr);
    Serial.println(F(" ..."));
    dump_byte_array(dataBlock, 16);
    Serial.println();
    status = (MFRC522::StatusCode)mfrc522.MIFARE_Write(blockAddr, dataBlock, 16);
    if (status != MFRC522::STATUS_OK)
    {
        Serial.print(F("MIFARE_Write() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    Serial.println();

    // Read data from the block (again, should now be what we have written)
    Serial.print(F("Reading data from block "));
    Serial.print(blockAddr);
    Serial.println(F(" ..."));
    status = (MFRC522::StatusCode)mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK)
    {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    Serial.print(F("Data in block "));
    Serial.print(blockAddr);
    Serial.println(F(":"));
    dump_byte_array(buffer, 16);
    Serial.println();

    // Check that data in block is what we have written
    // by counting the number of bytes that are equal
    Serial.println(F("Checking result..."));
    byte count = 0;
    for (byte i = 0; i < 16; i++)
    {
        // Compare buffer (= what we've read) with dataBlock (= what we've written)
        if (buffer[i] == dataBlock[i])
            count++;
    }
    Serial.print(F("Number of bytes that match = "));
    Serial.println(count);
    if (count == 16)
    {
        Serial.println(F("Success :-)"));
    }
    else
    {
        Serial.println(F("Failure, no match :-("));
        Serial.println(F("  perhaps the write didn't work properly..."));
    }
    Serial.println();

    // Dump the sector data
    Serial.println(F("Current data in sector:"));
    mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector);
    Serial.println();

    // Halt PICC
    mfrc522.PICC_HaltA();
    // Stop encryption on PCD
    mfrc522.PCD_StopCrypto1();
}

/*
Check firmware only once at startup
 */
void rfidCheckFirmware()
{
    Serial.println(F("*****************************"));
    Serial.println(F("MFRC522 Digital self test"));
    Serial.println(F("*****************************"));
    mfrc522.PCD_DumpVersionToSerial(); // Show version of PCD - MFRC522 Card Reader
    Serial.println(F("-----------------------------"));
    Serial.println(F("Only known versions supported"));
    Serial.println(F("-----------------------------"));
    Serial.println(F("Performing test..."));
    bool result = mfrc522.PCD_PerformSelfTest(); // perform the test
    Serial.println(F("-----------------------------"));
    Serial.print(F("Result: "));
    if (result)
        Serial.println(F("OK"));
    else
        Serial.println(F("DEFECT or UNKNOWN"));
    Serial.println();
}

/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize)
{
    for (byte i = 0; i < bufferSize; i++)
    {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}

/*
Helper routine to dump a byte array as hex values to Serial. 
*/
void printHex(byte *buffer, byte bufferSize)
{
    for (byte i = 0; i < bufferSize; i++)
    {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}

/*
Helper routine to dump a byte array as dec values to Serial.
*/
void printDec(byte *buffer, byte bufferSize)
{
    for (byte i = 0; i < bufferSize; i++)
    {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], DEC);
    }
}

/*
reconnect
Connexion server MQTT
id -> nom du client 
topic -> nom du topic pour envoyer les messages (domoticz/in)
*/
void reconnect(const char *id, const char *topic)
{
    //Boucle jusqu'à obtenir une reconnexion
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
    Serial.print("\nMessage arrived [");
    Serial.print(topic);
    Serial.println("] ");

    // decode payload message
    for (int i = 0; i < length; i++)
    {
        messageReceived += ((char)payload[i]);
    }
    // display incoming message
    Serial.println("Message recu:");
    Serial.print(messageReceived);

    // if domoticz message
    if (strcmp(topic, TOPIC_DOMOTICZ_OUT) == 0)
    {
        DeserializationError error = deserializeJson(jsonBuffer, messageReceived);
        if (error)
        {
            Serial.print(F("parsing Domoticz/out JSON Received Message failed with code: "));
            Serial.println(error.c_str());
            return;
        }

        int idx = jsonBuffer["idx"];
        int nvalue = jsonBuffer["nvalue"];
        float svalue = jsonBuffer["svalue"];
        const char *name = jsonBuffer["name"];

#ifdef DEBUG
        Serial.printf("\nIDX: %i, nVALUE: %i, sVALUE: %f, name: %s", idx, nvalue, float(svalue), name);
#endif
        cmdPinRelay(idx, nvalue, name);
    }
}

/*

*/
void cmdPinRelay(int idx, int nvalue, const char *name)
{

    if (idx == IDXESP52RELAIS)
    {
        Serial.printf("\nidx: %i, name: %s, nvalue: %i\n", idx, name, nvalue);
        if (nvalue == 0)
            digitalWrite(PIN_RELAIS, LOW);
        else if (nvalue == 1)
            digitalWrite(PIN_RELAIS, HIGH);
        else
        {
            Serial.println("\nErreur dans le message de commande du relais ");
        }
        Serial.printf("\nLe relais %s est a %i\n", name, digitalRead(PIN_RELAIS));
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
askMqttToDomoticz
Mise à jour valeur domoticz par messages MQTT 
idx -> adresse IDX domoticz du materiel
svalue -> donnée converti en String à envoyer 
topic -> topic pour envoyer les message(domoticz/in)
*/
void askMqttToDomoticz(int idx, String svalue, const char *topic)
{
    char msgToPublish[MQTT_MAX_PACKET_SIZE];

    StaticJsonDocument<1024> doc;
    doc["idx"] = idx;
    doc["command"] = svalue;
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