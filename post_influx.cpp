#include "Arduino.h"

// private credentials for network and external services
#include "secrets.h"

#include <InfluxDbClient.h>
// InfluxDB setup.  See config.h and secrets.h for site-specific settings

// InfluxDB client instance for InfluxDB 1
InfluxDBClient dbclient(INFLUXDB_URL, INFLUXDB_DB_NAME);

// Post data to Influx DB using the connection established during setup
// Operates over the network, so may take uint16_ta while to execute.
void post_influx(float pm25, float aqi, float tempF, float vocIndex, float humidity)
{
  Serial.println("Saving data to Influx");
  // Set InfluxDB 1 authentication params using values defined in secrets.h
  dbclient.setConnectionParamsV1(INFLUXDB_URL, INFLUXDB_DB_NAME, INFLUXDB_USER, INFLUXDB_PASSWORD);
  
  // InfluxDB Data points, binds to InfluxDB 'measurement' to use for data
  Point dbenvdata("weather");  // Weather data

  // Add constant Influx data point tags - only do once, will be added to all individual data points
  // Modify if required to reflect your InfluxDB data model (and set values in config.h)
  // Tags for environmental data (weather):
  dbenvdata.addTag("device", DEVICE_NAME);
  dbenvdata.addTag("location", DEVICE_LOCATION);
  dbenvdata.addTag("site", DEVICE_SITE);

  // If confirmed connection to InfluxDB server, store our data values (with retries)
  boolean dbsuccess = false;
  uint8_t dbtries;
  for (dbtries = 1; dbtries <= 5; dbtries++) {
    if (dbclient.validateConnection()) {
      Serial.println("Connected to InfluxDB: " + dbclient.getServerUrl());
      dbsuccess = true;
      break;
    }
    delay(dbtries * 10000); // Waiting longer each time we check for status
  }
  if(dbsuccess == false) {
    Serial.println("InfluxDB connection failed: " + dbclient.getLastErrorMessage());
    return;
  }
  else {
    // Connected, so store measured values into timeseries data point
    // First, the environmental data (weather)
    dbenvdata.clearFields();
    // Report sensor readings
    dbenvdata.addField("pm25", pm25);
    dbenvdata.addField("aqi", aqi);
    dbenvdata.addField("vocindex", vocIndex);
    dbenvdata.addField("humidity",humidity);
    dbenvdata.addField("temperature",tempF);
    Serial.println("Writing: " + dbclient.pointToLineProtocol(dbenvdata));
    // Write point via connection to InfluxDB host
    if (!dbclient.writePoint(dbenvdata)) {
      Serial.println("InfluxDB weather write failed: " + dbclient.getLastErrorMessage());
    }
  }
}
