/*
  Project Name:   PM2.5
  Description:    Regularly sample and log PM 2.5 levels

  See README.md for target information and revision history
*/

// Choose the right WiFi package, either ESP32 or ESP8266.  Provides access
// to the WiFiClient object class and accessor.  May not be needed for all
// network service access where service-specific libraries encapsulate it.

// ESP32 WiFi
//#include <WiFi.h>

// ESP8266 WiFi
#include <ESP8266WiFi.h>

// When is the onboard LED (LED_BUILTIN) on?  
#define LED_BUILTIN_ON    LOW
#define LED_BUILTIN_OFF   HIGH

// Step 1: Set conditional compile flags
#define DEBUG     // Output to serial port
#define MQTT        // log sensor data to MQTT broker
//#define HASSIO_MQTT  // And, if MQTT enabled, with Home Assistant too?
#define INFLUX      // Log data to InfluxDB server
// #define DWEET       // Log data to Dweet service
// #define THINGSPEAK  // Log data to ThingSpeak

// Simulate SEN5x sensor operations, e.g. for testing or easier development.  Returns random
// but plausible values for sensor readings.
//#define SIMULATE_SENSOR

// Sensor(s) are sampled at one interval, generally fairly often.  Readings
// are averaged and reported at a longer interval.  Configure that behavior here,
// allowing for more frequent processing when in DEBUG mode.
#ifdef DEBUG
  #define SAMPLE_INTERVAL 5   // sample interval for sensor in seconds
  #define REPORT_INTERVAL 1   // Interval at which samples are averaged & reported in minutes)
#else
  #define SAMPLE_INTERVAL 30  // sample interval for sensor in seconds
  #define REPORT_INTERVAL 15 // Interval at which samples are averaged & reported in minutes)
#endif

// To allow for varying network singal strength and responsiveness, make multiple
// attempts to connect to internet services at a measured interval.  If your environment
// is more challenged you may want to allow for more connection attempts and/or a longer
// delay between connection attempts.
const int CONNECT_ATTEMPT_LIMIT = 3;      // max connection attempts to internet services
const int CONNECT_ATTEMPT_INTERVAL = 5;  // seconds between internet service connect attempts

// set client ID; used by mqtt and wifi
#define CLIENT_ID "PM25"

#ifdef MQTT
  // Define MQTT topics used to publish sensor readings and device attributes.
  // Representative structure: username/feeds/groupname/feedname or username/feeds/feedname
  // e.g. #define MQTT_PUB_TEMPF   "sircoolio/feeds/office/tempf"

  // Default structure: site/room/device/data 
  #define MQTT_PUB_PM25       "7828/demo/pm25/pm25"
  #define MQTT_PUB_AQI        "7828/demo/pm25/aqi"
  #define MQTT_PUB_TEMPF      "7828/demo/pm25/tempf"
  #define MQTT_PUB_VOC        "7828/demo/pm25/vocindex"
  #define MQTT_PUB_HUMIDITY   "7828/demo/pm25/humidity"
  #define MQTT_PUB_RSSI       "7828/demo/pm25/rssi"

  // Additional state topic if integrating with Home Assistant
  #ifdef HASSIO_MQTT
    // Home Assistant integration wants all reported values published in one JSON
    // payload to a single "state" topic. See Home Assistant documentation for more details.
    // NOTE: MUST MATCH value used in Home Assistant MQTT configuration file 
    // (configuration.yaml). See hassio_mqtt.cpp for details.
    #define MQTT_HASSIO_STATE   "homeassistant/sensor/aqi-1/state"
  #endif
#endif

#ifdef INFLUX  
  // Name of Measurements expected/used in the Influx DB.
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging device data (e.g. battery)
  
  // Standard set of tag values used for each sensor data point stored to InfluxDB.  Reuses
  // CLIENT_ID as defined anove here in config.h as well as device location (e.g., room in 
  // the house) and site (indoors vs. outdoors, typically).


  // Tag data reported to InfluxDB to facilitate retrieval by query later
  // NAME for this device, should be unique
  #define DEVICE_NAME "pm25-test"

  // TYPE conveys the kind or class of device.  A location may have multiple devices of a
  // particular type, so name would be unique but type would not.
  #define DEVICE_TYPE "pm25"

  // Where is the device located?  Generally would the name of a room in a house or
  // building, e.g. "kitchen", "cellar", "workshop", etc.
  #define DEVICE_LOCATION "PM25-test"

  // SITE is typically indoor/outdoor or similar aspect apart from device LOCATION.
  // Can help group devices in ways that go beyond room/location.
  #define DEVICE_SITE "indoor"

#endif

// Post data to the internet via dweet.io.  Set DWEET_DEVICE to be a
// unique name you want associated with this reporting device, allowing
// data to be easily retrieved through the web or Dweet's REST API.
#ifdef DWEET
  #define DWEET_HOST "dweet.io"   // Typically dweet.io
  #define DWEET_DEVICE "makerhour-airquality"  // Must be unique across all of dweet.io
#endif