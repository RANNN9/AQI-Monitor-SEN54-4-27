// Port of code from Air Quality, modified for AQI use

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Post a dweet to report sensor data readings.  This routine blocks while
// talking to the network, so may take a while to execute.

#ifdef DWEET

  // Shared helper function(s)
  extern void debugMessage(String messageText, int messageLevel);

  // David's original PM2.5 dweet code
  const char* dweet_host = "dweet.io";
  void post_dweet(float pm25, float minaqi, float maxaqi, float aqi, float tempF, float vocIndex, float humidity)
  {
    WiFiClient dweet_client;

    if(WiFi.status() != WL_CONNECTED) {
      debugMessage("Lost network connection to " + String(WIFI_SSID) + "!",1);
      return;
    }
    
    debugMessage("Connecting to " + String(dweet_host) + " as " + String(DWEET_DEVICE),1);
        
    // Use our WiFiClient to connect to dweet
    if (!dweet_client.connect(dweet_host, 80)) {
      debugMessage("Connection failed!",1);
      return;
    }

    long rssi = WiFi.RSSI();
    IPAddress ip = WiFi.localIP();

    // Use HTTP post and send a data payload as JSON
    
    String postdata = "{\"wifi_rssi\":\""     + String(rssi)          + "\"," +
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
    dweet_client.println("User-Agent: ESP32/ESP8266 (orangemoose)/1.0");
    dweet_client.println("Cache-Control: no-cache");
    dweet_client.println("Content-Type: application/json");
    dweet_client.print("Content-Length: ");
    dweet_client.println(postdata.length());
    dweet_client.println();
    dweet_client.println(postdata);
    debugMessage("Dweet POST:",1);
    debugMessage(postdata,1);

    delay(1500);  

    // Read all the lines of the reply from server (if any) and print them to Serial Monitor
    #ifdef DEBUG
      debugMessageln("Dweet server response:",1);
      while(dweet_client.available()){
        String line = dweet_client.readStringUntil('\r');
        debugMessage(line,1);
      }
      debugMessageln("-----",1);
    #endif
    
    // Close client connection to dweet server
    dweet_client.stop();
  }
#endif