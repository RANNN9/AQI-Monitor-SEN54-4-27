// Port of code from Air Quality, modified for AQI use

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// ESP32 WiFi
#include <WiFi.h>

//ESP8266 WiFi
//#include <ESP8266WiFi.h>

// Post a dweet to report sensor data readings.  This routine blocks while
// talking to the network, so may take a while to execute.

#ifdef DWEET

  // WiFi client object shared with main() routine
  //extern WiFiClient client;  // (Does this have to be global/shared??)

  // David's original PM2.5 dweet code
  const char* dweet_host = "dweet.io";
  void post_dweet(float pm25, float minaqi, float maxaqi, float aqi, float tempF, float vocIndex, float humidity)
  {
    WiFiClient dweet_client;

    if(WiFi.status() != WL_CONNECTED) {
      Serial.print("Lost network connection to '");
      Serial.print(WIFI_SSID);
      Serial.println("'!");
      return;
    }
    
    Serial.print("connecting to ");
    Serial.print(dweet_host);
    Serial.print(" as ");
    Serial.println(String(DWEET_DEVICE));
        
    // Use our WiFiClient to connect to dweet
    if (!dweet_client.connect(dweet_host, 80)) {
      Serial.println("connection failed");
      return;
    }

    long rssi = WiFi.RSSI();
    IPAddress ip = WiFi.localIP();

    // Use HTTP post and send a data payload as JSON
    
    String postdata = "{\"wifi_rssi\":\""     + String(rssi)           + "\"," +
                      "\"AQI\":\""           + String(aqi,2)          + "\"," +
                      "\"address\":\""       + ip.toString()          + "\"," +
                      "\"temperature\":\""   + String(tempF,1)        + "\"," +
                      "\"vocIndex\":\""      + String(vocIndex,1)     + "\"," +
                      "\"humidity\":\""      + String(humidity,1)     + "\"," +
                      "\"PM25_value\":\""    + String(pm25,2)         + "\"," +
                      "\"min_AQI\":\""       + String(minaqi,2)       + "\"," + 
                      "\"max_AQI\":\""       + String(maxaqi,2)       + "\"}";
    // Note that the dweet device 'name' gets set here, is needed to fetch values
    dweet_client.println("POST /dweet/for/" + String(DWEET_DEVICE) + " HTTP/1.1");
    dweet_client.println("Host: dweet.io");
    dweet_client.println("User-Agent: ESP8266 (orangemoose)/1.0");
    dweet_client.println("Cache-Control: no-cache");
    dweet_client.println("Content-Type: application/json");
    dweet_client.print("Content-Length: ");
    dweet_client.println(postdata.length());
    dweet_client.println();
    dweet_client.println(postdata);
    Serial.println(postdata);

    delay(1500);  
    // Read all the lines of the reply from server (if any) and print them to Serial Monitor
    while(dweet_client.available()){
      String line = dweet_client.readStringUntil('\r');
      Serial.print(line);
    }
    
    Serial.println("closing connection");
    Serial.println();
  }
#endif