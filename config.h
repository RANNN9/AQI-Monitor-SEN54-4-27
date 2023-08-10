/*
  Project:      pm25
  Description:  public (non-secret) configuration data for pm25
*/

// Configuration Step 1: Set debug message output
// comment out to turn off; 1 = summary, 2 = verbose
// #define DEBUG 1

// Configuration Step 2: Set network data endpoints
// #define MQTT     // log sensor data to MQTT broker
// #define HASSIO_MQTT  // And, if MQTT enabled, with Home Assistant too?
#define INFLUX // Log data to InfluxDB server
// #define DWEET       // Log data to Dweet service
// #define THINGSPEAK  // Log data to ThingSpeak

// Configuration Step 3: Select environment sensor read intervals

// Simulate SEN5x sensor operations, e.g. for testing or easier development.  Returns random
// but plausible values for sensor readings.
//#define SIMULATE_SENSOR

// Sensor(s) are sampled at one interval, generally fairly often.  Readings
// are averaged and reported at a longer interval.  Configure that behavior here,
// allowing for more frequent processing when in DEBUG mode.
#ifdef DEBUG
  #define SAMPLE_INTERVAL 30  // sensor sample interval in seconds
  #define REPORT_INTERVAL 2   // sample averaging and reporting interval in minutes
#else
  #define SAMPLE_INTERVAL 60
  #define REPORT_INTERVAL 30
#endif

// Configuration Step 4: Set screen parameters, if desired
#define  SCREEN    // use screen as output

// Pin config for display
#ifdef SCREEN
  #define TFT_CS         16
  #define TFT_RST        15                                            
  #define TFT_DC         0

  // rotation 1 orients the display so the pins are at the bottom of the display
  // rotation 2 orients the display so the pins are at the top of the display
  // rotation of 3 flips it so the wiring is on the left side of the display
  #define DISPLAY_ROTATION 3

  // screen layout assists in pixels
  const int xMargins = 5;
  const int yMargins = 2;
  const int wifiBarWidth = 3;
  const int wifiBarHeightIncrement = 3;
  const int wifiBarSpacing = 5;
  const int yTemperature = 23;
  const int yLegend = 95;
  const int legendHeight = 10;
  const int legendWidth = 5; 
#endif

// Configuration Step 5: Set network data endpoint parameters, if applicable
// set client ID; used by mqtt and wifi
// structure is PM25_room-name; e.g. PM25_kitchen
#define CLIENT_ID "PM25_kitchen"

#ifdef INFLUX  
  // Name of Measurements expected/used in the Influx DB.
  #define INFLUX_ENV_MEASUREMENT "weather"  // Used for environmental sensor data
  #define INFLUX_DEV_MEASUREMENT "device"   // Used for logging device data (e.g. battery)
#endif

// Post data to the internet via dweet.io.  Set DWEET_DEVICE to be a
// unique name you want associated with this reporting device, allowing
// data to be easily retrieved through the web or Dweet's REST API.
#ifdef DWEET
  #define DWEET_HOST "dweet.io"   // Typically dweet.io
  #define DWEET_DEVICE "makerhour-pm25"  // Must be unique across all of dweet.io
#endif

// Configuration variables that are less likely to require changes

// To allow for varying network singal strength and responsiveness, make multiple
// attempts to connect to internet services at a measured interval.  If your environment
// is more challenged you may want to allow for more connection attempts and/or a longer
// delay between connection attempts.
#define CONNECT_ATTEMPT_LIMIT 3     // max connection attempts to internet services
#define CONNECT_ATTEMPT_INTERVAL 10 // seconds between internet service connect attempts

// Sleep time in seconds if hardware error occurs
#define HARDWARE_ERROR_INTERVAL 10