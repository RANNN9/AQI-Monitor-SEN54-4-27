/*
 * Additional routines for use in an MQTT-enabled environment with Home Assistant, allowing
 * sensor readings to be reported to Home Assistant.   
 * 
 * Requires the following addition to configuration.yaml for Home Assistant, with the
 * state topic declared to match MQTT_HASSIO_STATE as defined in config.h. Also, Unique IDs, 
 * which enable additional UI customization features for the sensors in Home Assistant,
 * can be generated here: https://www.uuidgenerator.net/version1.

   mqtt:
    sensor:
     - name: "Temperature"
      device_class: "temperature"
      state_topic: "homeassistant/sensor/aqi-1/state"
      unit_of_measurement: "°F"
      unique_id: "-- GENERATE A UUID TO USE HERE --"
      value_template: "{{ value_json.temperature }}"
    - name: "Humidity"
      device_class: "humidity"
      state_topic: "homeassistant/sensor/aqi-1/state"
      unit_of_measurement: "%"
      unique_id: "-- GENERATE A UUID TO USE HERE --"
      value_template: "{{ value_json.humidity }}"
    - name: "PM2.5"
      device_class: "pm25"
      state_topic: "homeassistant/sensor/aqi-1/state"
      unit_of_measurement: "µg/m³"
      unique_id: "-- GENERATE A UUID TO USE HERE --"
      value_template: "{{ value_json.pm25 }}"
    - name: "AQI"
      device_class: "aqi"
      state_topic: "homeassistant/sensor/aqi-1/state"
      unit_of_measurement: "AQI"
      unique_id: "-- GENERATE A UUID TO USE HERE --"
      value_template: "{{ value_json.aqi }}"
 */

#include "Arduino.h"

// hardware and internet configuration parameters
#include "config.h"
// private credentials for network, MQTT, weather provider
#include "secrets.h"

// Shared helper function
  extern void debugMessage(String messageText, int messageLevel);

#if defined MQTT && defined HASSIO_MQTT
    // MQTT setup
    #include <Adafruit_MQTT.h>
    #include <Adafruit_MQTT_Client.h>
    #include <ArduinoJson.h>
    extern Adafruit_MQTT_Client pm25_mqtt;

    // Called to publish sensor readings as a JSON payload, as part of overall MQTT
    // publishing as implemented in post_mqtt().  Should only be invoked if 
    // Home Assistant MQTT integration is enabled in config.h.
    // Note that it depends on the value of the state topic matching what's in Home
    // Assistant's configuration file (configuration.yaml).
    void hassio_mqtt_publish(float pm25, float aqi, float tempF, float vocIndex, float humidity) {
        const int capacity = JSON_OBJECT_SIZE(5);
        StaticJsonDocument<capacity> doc;

        // Declare buffer to hold serialized object
        char output[1024];
        
        Adafruit_MQTT_Publish rco2StatePub = Adafruit_MQTT_Publish(&pm25_mqtt,MQTT_HASSIO_STATE);

        debugMessage(String("Publishing AQI values to Home Assistant via MQTT, topic: ") + MQTT_HASSIO_STATE,1);
        doc["temperatureF"] = tempF;
        doc["humidity"] = humidity;
        doc["aqi"] = aqi;
        doc["pm25"] = pm25;
        doc["voc"] = vocIndex;

        serializeJson(doc,output);
        // Publish state info to its topic (MQTT_HASSIO_STATE)
        debugMessage(output,1);
        rco2StatePub.publish(output);
    }
#endif