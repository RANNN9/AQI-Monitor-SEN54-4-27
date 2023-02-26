// Copy this file to "secrets.h" and adjust the access credentials here to
// match your deployment environment.

// // WiFi configuration settings
// #define WIFI_SSID "YOUR_WIFI_SSID"
// #define WIFI_PASS "YOUR_WIFI_PASSWORD"

// MQTT configuration settings
// #ifdef MQTT
//  #define MQTT BROKER    "mqtt.hostname.local or IP address"
// 	#define MQTT_PORT  			port_number	// use 8883 for SSL
// 	#define MQTT_USER			 "key_value"
//  #define MQTT_PASSWORD  "key_value"
// #endif

// // Device configuration info for ThingSpeak
// #define THINGS_CHANID 9999999           // ThingSpeak channel ID
// #define THINGS_APIKEY "CHANNEL_WRITE_APIKEY"// write API key for ThingSpeak Channel


// InfluxDB access information, varies depending on whether target server is
// running v1 or v2 of InfluxDB.  Configure as appropriate.

// If server runs Influx v1.X, configure here
// InfluxDB server url using name or IP address (not localhost)
// #define INFLUXDB_URL "http://influxdbhost.local:8086"
// InfluxDB v1 database name 
// #define INFLUXDB_DB_NAME "home"
// InfluxDB v1 user name
// #define INFLUXDB_USER "GRAFANA_USER"
// InfluxDB v1 user password
// #define INFLUXDB_PASSWORD "GRAFANA_PASSWORD"

// If server runs InfluxDB v2.X, configure here
// InfluxDB 2 server url, e.g. http://192.168.1.48:8086 (Use: InfluxDB UI -> Load Data -> Client Libraries)
// #define INFLUXDB_URL "http://influxdbhost.local:8086"
// InfluxDB 2 server or cloud API authentication token (Use: InfluxDB UI -> Load Data -> Tokens -> <select token>)
// #define INFLUXDB_TOKEN "--InfluxDB--access-token-value--"
// InfluxDB 2 organization name or id (Use: InfluxDB UI -> Settings -> Profile -> <name under tile> )
// #define INFLUXDB_ORG "influxdborg"
// InfluxDB 2 bucket name (Use: InfluxDB UI -> Load Data -> Buckets)
// #define INFLUXDB_BUCKET "influxdbbucket"