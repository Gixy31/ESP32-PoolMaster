// Firmware revisions
#define FIRMW "ESP-3.0"
#define TFT_FIRMW "TFT-2.0"						   

#define DEBUG_LEVEL DBG_INFO     // Possible levels : NONE/ERROR/WARNING/INFO/DEBUG/VERBOSE

//Version of config stored in EEPROM
//Random value. Change this value (to any other value) to revert the config to default values
#define CONFIG_VERSION 50
// WiFi credentials
#define WIFI_NETWORK "YOUR_WIFI_NETWORK_ID"
#define WIFI_PASSWORD "YOUR_WIFI_NETWORK_PWD"
#define OTA_PWDHASH   "Your_OTA_password_hash"

//IFTTT key to trigger event
#define IFTTT_key "/trigger/PoolMaster/with/key/Your_IFTTT_Key"

// PID Directions (either DIRECT or REVERSE depending on Ph/Orp correction vs water properties)
#define PhPID_DIRECTION REVERSE
#define OrpPID_DIRECTION DIRECT

#define FILTRATION_PUMP 32
#define ROBOT_PUMP	    33
#define PH_PUMP         25
#define CHL_PUMP        26
#define RELAY_R0        27   // Projecteur
#define RELAY_R1         4   // Spare, not connected

//Digital input pins connected to Acid and Chl tank level reed switches
#define CHL_LEVEL       39   // not wired. Use NO_LEVEL option of Pump class
#define PH_LEVEL        36   //                - " -

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


//MQTT stuff including local broker/server IP address, login and pwd
//------------------------------------------------------------------
//interval (in miilisec) between MQTT publishes of measurement data
#define PUBLISHINTERVAL 30000

#define MQTT_SERVER_IP IPAddress(000, 000, 000, 000)
#define MQTT_SERVER_PORT 1883

//#define MQTT_LOGIN                           // uncomment if MQTT broker needs login/pwd
//#define MQTT_SERVER_ID    "ESP32Pool"		   // MQTT server ID
//#define MQTT_SERVER_LOGIN "Your_Login"
//#define MQTT_SERVER_PWD   "Your_Pwd" 				

// Topic used in DEVT or OPER mode

#ifdef DEVT
  #define POOLTOPIC "Home/Pool6/"
#else
  #define POOLTOPIC "Home/Pool/"
#endif 

// Robot pump timing
#define ROBOT_DELAY 60     // Robot start delay after filtration in mn
#define ROBOT_DURATION 90  // Robot cleaning duration

//Display timeout before blanking
//-------------------------------
#define TFT_SLEEP 60000L 

// Loop tasks scheduling parameters
//---------------------------------
// T1: AnalogPoll
// T2: PoolServer
// T3: PoolMaster
// T4: getTemp
// T5: OrpRegulation
// T6: pHRegulation
// T7: StatusLights
// T8: PublishMeasures
// T9: PublishSettings 

//Periods 
// Task9 period is initialized with PUBLISHINTERVAL and can be changed dynamically
#define PT1 125
#define PT2 500
#define PT3 500
#define PT4 1000 / (1 << (12 - TEMPERATURE_RESOLUTION))
#define PT5 1000
#define PT6 1000
#define PT7 3000
#define PT8 30000

//Start offsets to spread tasks along time
// Task1 has no delay
#define DT2 190/portTICK_PERIOD_MS
#define DT3 310/portTICK_PERIOD_MS
#define DT4 440/portTICK_PERIOD_MS
#define DT5 560/portTICK_PERIOD_MS
#define DT6 920/portTICK_PERIOD_MS
#define DT7 100/portTICK_PERIOD_MS
#define DT8 570/portTICK_PERIOD_MS
#define DT9 940/portTICK_PERIOD_MS

//#define CHRONO                    // Activate tasks timings traces for profiling
//#define SIMU                      // Used to simulate pH/ORP sensors. Very simple simulation:
                                    // the sensor value is computed from the output of the PID 
                                    // loop to reach linearly the theorical value produced by this
                                    // output after one hour