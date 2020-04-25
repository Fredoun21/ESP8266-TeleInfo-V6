/*
setup_wifi
Connexion du module ESP au wifi
local_ip -> adressi IP du module
gateway -> passerelle réseau
subnet -> masque de sous réseau
ssid -> nom du SSID pour la connexion Wifi
password -> mot de passe pour la connexion Wifi 
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "connect.h"

/*
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
*/