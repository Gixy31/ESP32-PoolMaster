#define FILTRATION_PUMP 25
#define PH_PUMP         26
#define CHL_PUMP        27
#define ROBOT_PUMP	    32

//One wire bus for the water temperature measurement
#define ONE_WIRE_BUS_A  19

//I2C bus for analog measurement with ADS1115 of pH, ORP and water pressure 
#define I2C_SDA			21
#define I2C_SCL			22

#define RELAY_R0         4   // Projecteur
#define RELAY_R1         5   // Spare, not connected
#define RELAY_R2         5   // non existing

//Digital input pins connected to Acid and Chl tank level reed switches
#define CHL_LEVEL       35
#define PH_LEVEL        36

#define WDT_TIMEOUT     10

// Server port
#define SERVER_PORT 8060

//12bits (0,06°C) temperature sensor resolution
#define TEMPERATURE_RESOLUTION 12

//Version of config stored in EEPROM
//Random value. Change this value (to any other value) to revert the config to default values
#define CONFIG_VERSION 3

#define _endl "\n"

//MQTT stuff including local broker/server IP address, login and pwd
//------------------------------------------------------------------
//interval (in miilisec) between MQTT publishes of measurement data
#define PUBLISHINTERVAL 30000

#define MQTT_SERVER_IP IPAddress(192, 168, 1, 51)
#define MQTT_SERVER_PORT 1883
