/*
  Project Name:   PM2.5
  Description:    write PM2.5 sensor data to InfluxDB

  See README.md for target information and revision history
*/

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// only compile if MQTT enabled
#ifdef MQTT

  // Shared helper function
  extern void debugMessage(String messageText, int messageLevel);

  #ifdef HASSIO_MQTT
    extern void hassio_mqtt_publish(float pm25, float aqi, float tempF, float vocIndex, float humidity);
  #endif

  // MQTT setup
  #include <Adafruit_MQTT.h>
  #include <Adafruit_MQTT_Client.h>
  extern Adafruit_MQTT_Client pm25_mqtt;

  void mqttConnect()
  // Connects and reconnects to MQTT broker, call as needed to maintain connection
  {  
    // exit if already connected
    if (pm25_mqtt.connected())
    {
      debugMessage(String("Already connected to MQTT broker ") + MQTT_BROKER,1);
      return;
    }
    
    int8_t mqttErr;

    // Attempts MQTT connection, and if unsuccessful, re-attempts after CONNECT_ATTEMPT_INTERVAL second delay for CONNECT_ATTEMPT_LIMIT times
    for(int tries = 1; tries <= CONNECT_ATTEMPT_LIMIT; tries++)
    {
      if ((mqttErr = pm25_mqtt.connect()) == 0)
      {
        debugMessage(String("Connected to MQTT broker ") + MQTT_BROKER);
        return;
      }
      pm25_mqtt.disconnect();
      debugMessage(String("MQTT connection attempt ") + tries + " of " + CONNECT_ATTEMPT_LIMIT + " failed with error msg: " + pm25_mqtt.connectErrorString(mqttErr),1);
      delay(CONNECT_ATTEMPT_INTERVAL*1000);
      // }
    }
  } 

  bool mqttDeviceWiFiUpdate(int rssi)
  {
    bool result = false;
    if (rssi!=0)
    {
      // Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&pm25_mqtt, MQTT_PUB_RSSI, MQTT_QOS_1); // if problematic, remove QOS parameter
      Adafruit_MQTT_Publish rssiLevelPub = Adafruit_MQTT_Publish(&pm25_mqtt, MQTT_PUB_RSSI);

      mqttConnect();

      if (rssiLevelPub.publish(rssi))
      {
        debugMessage("MQTT publish: WiFi RSSI succeeded",1);
        result = true;
      }
      else
      {
        debugMessage("MQTT publish: WiFi RSSI failed",1);
      }
      //pm25_mqtt.disconnect();
    }
    return(result);
  }
  
  bool mqttSensorUpdate(float pm25, float aqi, float tempF, float vocIndex, float humidity)
  // Publishes sensor data to MQTT broker
  {
    bool result = false;

    // add ,MQTT_QOS_1); if problematic, remove QOS parameter
    Adafruit_MQTT_Publish tempfPub = Adafruit_MQTT_Publish(&pm25_mqtt, MQTT_PUB_TEMPF);
    Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&pm25_mqtt, MQTT_PUB_HUMIDITY);
    Adafruit_MQTT_Publish vocPub = Adafruit_MQTT_Publish(&pm25_mqtt, MQTT_PUB_VOC);
    Adafruit_MQTT_Publish pm25Pub = Adafruit_MQTT_Publish(&pm25_mqtt, MQTT_PUB_PM25);
    Adafruit_MQTT_Publish aqiPub = Adafruit_MQTT_Publish(&pm25_mqtt, MQTT_PUB_AQI);

    mqttConnect();

    // Attempt to publish sensor data
    if(pm25Pub.publish(pm25))
    {
      debugMessage("MQTT publish: PM2.5 succeeded",1);
      debugMessage(MQTT_PUB_PM25);
      result = true;
    }
    else {
      debugMessage("MQTT publish: PM2.5 failed",1);
    }
    
    if(aqiPub.publish(aqi))
    {
      debugMessage("MQTT publish: AQI succeeded",1);
      result = true;
    }
    else {
      debugMessage("MQTT publish: AQI failed",1);
      result = false;
    }

    if(vocPub.publish(vocIndex))
    {
      debugMessage("MQTT publish: VOC index succeeded",1);
      result = true;
    }
    else {
      debugMessage("MQTT publish: VOC index failed",1);
    }

    if(tempfPub.publish(tempF))
    {
      debugMessage("MQTT publish: Temperature succeeded",1);
      result = true;
    }
    else {
      debugMessage("MQTT publish: Temperature failed",1);
    }

    if(humidityPub.publish(humidity))
    {
      debugMessage("MQTT publish: Humidity succeeded",1);
      result = true;
    }
    else {
      debugMessage("MQTT publish: Humidity failed",1);
    }

    #ifdef HASSIO_MQTT
      // Either configure sensors in Home Assistant's configuration.yaml file
      // directly or attempt to do it via MQTT auto-discovery
      // hassio_mqtt_setup();  // Config for MQTT auto-discovery
      hassio_mqtt_publish(pm25,aqi,tempF,vocIndex,humidity);
    #endif
    return(result);
  }
#endif