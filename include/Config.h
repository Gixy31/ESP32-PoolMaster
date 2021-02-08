#define FILTRATION_PUMP 32
#define ROBOT_PUMP	    33
#define PH_PUMP         25
#define CHL_PUMP        26
#define RELAY_R0        27   // Projecteur
#define RELAY_R1         4   // Spare, not connected
#define RELAY_R2         4   // non existing

//Digital input pins connected to Acid and Chl tank level reed switches
#define CHL_LEVEL       39
#define PH_LEVEL        36

//One wire bus for the air/water temperature measurement
#define ONE_WIRE_BUS_A  18
#define ONE_WIRE_BUS_W  19

//I2C bus for analog measurement with ADS1115 of pH, ORP and water pressure 
//and status LED through PCF8574A 
#define I2C_SDA			21
#define I2C_SCL			22

// Buzzer
#define BUZZER           2

#define WDT_TIMEOUT     10

// Server port
#define SERVER_PORT 8060

//OTA port
#define OTA_PORT    8063

//12bits (0,06°C) temperature sensors resolution
#define TEMPERATURE_RESOLUTION 12

//Version of config stored in EEPROM
//Random value. Change this value (to any other value) to revert the config to default values
#define CONFIG_VERSION 2

#define _endl "\n"

//MQTT stuff including local broker/server IP address, login and pwd
//------------------------------------------------------------------
//interval (in miilisec) between MQTT publishes of measurement data
#define PUBLISHINTERVAL 30000

#define MQTT_SERVER_IP IPAddress(192, 168, 1, 51)
#define MQTT_SERVER_PORT 1883

//Display timeout before blanking
#define TFT_SLEEP 60000 