// MQTT related functions for PoolMaster
// Use JSON version 6

#undef __STRICT_ANSI__
#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"

AsyncMqttClient mqttClient;

bool MQTTConnection = false;           // Status of connection to broker
TimerHandle_t mqttReconnectTimer;      // Reconnect timer

//Queue object to store outgoing JSON messages (up to 7)
//buffers for MQTT string payload
#define PAYLOAD_BUFFER_LENGTH 150
char Payload[PAYLOAD_BUFFER_LENGTH];

// BitMaps with GPIO states
uint8_t BitMap1 = 0;
uint8_t BitMap2 = 0;

//const char* MqttServerIP = "broker.mqttdashboard.com"; //cloud-based MQTT broker to test when node-red and MQTT broker are not installed locally (/!\ public and unsecure!)
const char* MqttServerClientID = "ESP32Pool";            // /!\ choose a client ID which is unique to this Arduino board
const char* MqttServerLogin    = nullptr;                //replace by const char* MqttServerLogin = nullptr; in case broker does not require a login/pwd
const char* MqttServerPwd      = nullptr;                //replace by const char* MqttServerPwd = nullptr; in case broker does not require a login/pwd
const char* PoolTopicMeas1     = "Home/Pool/Meas1";
const char* PoolTopicMeas2     = "Home/Pool/Meas2";
const char* PoolTopicSet1      = "Home/Pool/Set1";
const char* PoolTopicSet2      = "Home/Pool/Set2";
const char* PoolTopicSet3      = "Home/Pool/Set3";
const char* PoolTopicSet4      = "Home/Pool/Set4";
const char* PoolTopicSet5      = "Home/Pool/Set5";
const char* PoolTopicAPI       = "Home/Pool/API";
const char* PoolTopicStatus    = "Home/Pool/status";
const char* PoolTopicError     = "Home/Pool/Err";

// Functions prototypes
void mqttInit(void);
void mqttErrorPublish(const char* );
void connectToMqtt(void);
void WiFiEvent(WiFiEvent_t );
void EncodeBitMap(void);
void onMqttConnect(bool);
void onMqttDisconnect(AsyncMqttClientDisconnectReason);
void onMqttSubscribe(uint16_t, uint8_t);
void onMqttUnSubscribe(uint16_t);
void onMqttMessage(char* , char* , AsyncMqttClientMessageProperties , size_t , size_t , size_t );
void onMqttPublish(uint16_t);
void PublishSettings(void);
void UpdateWiFi(bool);
int  freeRam(void);

void mqttInit() {
  //Init Async MQTT
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnSubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_SERVER_IP,MQTT_SERVER_PORT);
} 

void mqttErrorPublish(const char* Payload){
  if (mqttClient.publish(PoolTopicError, 1, true, Payload) !=0)
  {
    Debug.print(DBG_WARNING,"Payload: %s - Payload size: %d",Payload, sizeof(Payload));
  }
  else
  {
    Debug.print(DBG_WARNING,"Unable to publish the following payload: %s",Payload);
  }
}    

void connectToMqtt(){
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event){
  switch(event){
    case SYSTEM_EVENT_STA_GOT_IP:
      Debug.print(DBG_INFO,"WiFi connected, connecting to MQTT");
      UpdateWiFi(true);
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      xTimerStop(mqttReconnectTimer,0);
      UpdateWiFi(false);
      break;    
    default:
      break;  
  }
}

//Encode digital inputs states into one Byte (more efficient to send over MQTT)
void EncodeBitMap()
{
  BitMap1 = 0;
  BitMap2 = 0;
  BitMap1 |= (FiltrationPump.IsRunning() & 1) << 7;
  BitMap1 |= (PhPump.IsRunning() & 1) << 6;
  BitMap1 |= (ChlPump.IsRunning() & 1) << 5;
  BitMap1 |= (PhPump.TankLevel() & 1) << 4;
  BitMap1 |= (ChlPump.TankLevel() & 1) << 3;
  BitMap1 |= (PSIError & 1) << 2;
  BitMap1 |= (PhPump.UpTimeError & 1) << 1;
  BitMap1 |= (ChlPump.UpTimeError & 1) << 0;

  BitMap2 |= (PhPID.GetMode() & 1) << 7;
  BitMap2 |= (OrpPID.GetMode() & 1) << 6;
  BitMap2 |= (storage.AutoMode & 1) << 5;
  BitMap2 |= (RobotPump.IsRunning() & 1) << 4;

  BitMap2 |= !digitalRead(RELAY_R0) << 3;
  BitMap2 |= !digitalRead(RELAY_R1) << 2;
  BitMap2 |= !digitalRead(RELAY_R2) << 1;
  BitMap2 |= (0 & 1U) << 0;      

}

// Once connected to MQTT broker, subscribe to the PoolTopicAPI topic in order to receive future commands
// then publish the "online" message on the "status" topic. If Ethernet connection is ever lost
// "status" will switch to "offline". Very useful to check that the Arduino is alive and functional
void onMqttConnect(bool sessionPresent){
  Debug.print(DBG_INFO,"Connected to MQTT, present session: %d",sessionPresent);
  mqttClient.subscribe(PoolTopicAPI,2);
  mqttClient.publish(PoolTopicStatus,1,true,"Online");
  MQTTConnection = true;
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason){
  Debug.print(DBG_WARNING,"Disconnected from MQTT");
  if(WiFi.isConnected()) xTimerStart(mqttReconnectTimer,0);
  MQTTConnection = false;
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos){
    Debug.print(DBG_INFO,"Subscribe ack., qos: %d",qos);
}

void onMqttUnSubscribe(uint16_t packetId){
    Debug.print(DBG_INFO,"unSubscribe ack.");
}

void onMqttPublish(uint16_t packetId){
    Debug.print(DBG_VERBOSE,"Publish ack., packetId: %d",packetId);
}

// MQTT callback
// This function is called when messages are published on the MQTT broker on the PoolTopicAPI topic to which we subscribed
// Add the received command to a message queue for later processing and exit the callback
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  //Pool commands. This check might be redundant since we only subscribed to this topic
  if (strcmp(topic,PoolTopicAPI)==0)
  {
    if (queueIn.enqueue(payload))
    {
      Debug.print(DBG_INFO,"Added command to queue: %s",payload);
    }
    else
    {
      Debug.print(DBG_ERROR,"Could not add command to queue, queue is full");
    }
    Debug.print(DBG_VERBOSE,"FreeRam: %d Queued messages: %d",freeRam(),queueIn.itemCount());
  }
}

// Publishes system settings to MQTT broker
void PublishSettings()
{
  if (mqttClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
    const int capacity = JSON_OBJECT_SIZE(9);
    StaticJsonDocument<capacity> root;

    root["FW"]     = Firmw;                            //firmware revision
    root["FSta"]   = storage.FiltrationStart;          //Computed filtration start hour, in the morning (hours)
    root["FStaM"]  = storage.FiltrationStartMin;       //Earliest Filtration start hour, in the morning (hours)
    root["FDu"]    = storage.FiltrationDuration;       //Computed filtration duration based on water temperature (hours)
    root["FStoM"]  = storage.FiltrationStopMax;        //Latest hour for the filtration to run. Whatever happens, filtration won't run later than this hour (hour)
    root["FSto"]   = storage.FiltrationStop;           //Computed filtration stop hour, equal to FSta + FDu (hour)
    root["Dpid"]   = storage.DelayPIDs;                //Delay from FSta for the water regulation/PIDs to start (mins)  
    root["pHUTL"]  = storage.PhPumpUpTimeLimit / 60;   //Max allowed daily run time for the pH pump (/!\ mins)
    root["ChlUTL"] = storage.ChlPumpUpTimeLimit / 60;  //Max allowed daily run time for the Chl pump (/!\ mins)

    if (root.size() < PAYLOAD_BUFFER_LENGTH)
    {
      serializeJson(root,Payload,sizeof(Payload));
      if (mqttClient.publish(PoolTopicSet1, 1, true, Payload) !=0)
      {
        Debug.print(DBG_DEBUG,"Payload: %s - Payload size: %d",Payload,root.size());
      }
      else
      {
        Debug.print(DBG_DEBUG,"Unable to publish the following payload: %s",Payload);
      }
    }
    else
    {
      Debug.print(DBG_ERROR,"MQTT Payload buffer overflow! - Payload size: %d",root.size());
    }
  }
  else
    Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

  if (mqttClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
    const int capacity = JSON_OBJECT_SIZE(8);
    StaticJsonDocument<capacity> root;

    root["pHWS"]  = storage.PhPIDWindowSize / 1000 / 60;        //pH PID window size (/!\ mins)
    root["ChlWS"] = storage.OrpPIDWindowSize / 1000 / 60;       //Orp PID window size (/!\ mins)
    root["pHSP"]  = storage.Ph_SetPoint * 100;                  //pH setpoint (/!\ x100)
    root["OrpSP"] = storage.Orp_SetPoint;                       //Orp setpoint
    root["WSP"]   = storage.WaterTemp_SetPoint * 100;           //Water temperature setpoint (/!\ x100)
    root["WLT"]   = storage.WaterTempLowThreshold * 100;        //Water temperature low threshold to activate anti-freeze mode (/!\ x100)
    root["PSIHT"] = storage.PSI_HighThreshold * 100;            //Water pressure high threshold to trigger error (/!\ x100)
    root["PSIMT"] = storage.PSI_MedThreshold * 100;             //Water pressure medium threshold (unused yet) (/!\ x100)

    if (root.size() < PAYLOAD_BUFFER_LENGTH)
    {
      serializeJson(root,Payload,sizeof(Payload));
      if (mqttClient.publish(PoolTopicSet2, 1, true, Payload) != 0)
      {
        Debug.print(DBG_DEBUG,"Payload: %s - Payload size: %d",Payload,root.size());
      }
      else
      {
        Debug.print(DBG_DEBUG,"Unable to publish the following payload: %s",Payload);
      }
    }
    else
    {
      Debug.print(DBG_ERROR,"MQTT Payload buffer overflow! - Payload size: %d",root.size());
    }
  }
  else
    Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

  if (mqttClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
    const int capacity = JSON_OBJECT_SIZE(6);
    StaticJsonDocument<capacity> root;

    root["pHC0"]  = storage.pHCalibCoeffs0;            //pH sensor calibration coefficient C0
    root["pHC1"]  = storage.pHCalibCoeffs1;            //pH sensor calibration coefficient C1
    root["OrpC0"] = storage.OrpCalibCoeffs0;           //Orp sensor calibration coefficient C0
    root["OrpC1"] = storage.OrpCalibCoeffs1;           //Orp sensor calibration coefficient C1
    root["PSIC0"] = storage.PSICalibCoeffs0;           //Pressure sensor calibration coefficient C0
    root["PSIC1"] = storage.PSICalibCoeffs1;           //Pressure sensor calibration coefficient C1

    if (root.size() < PAYLOAD_BUFFER_LENGTH)
    {
      serializeJson(root,Payload,sizeof(Payload));
      if (mqttClient.publish(PoolTopicSet3, 1, true, Payload) != 0)
      {
        Debug.print(DBG_DEBUG,"Payload: %s - Payload size: %d",Payload,root.size());
      }
      else
      {
        Debug.print(DBG_DEBUG,"Unable to publish the following payload: %s",Payload);
      }
    }
    else
    {
      Debug.print(DBG_ERROR,"MQTT Payload buffer overflow! - Payload size: %d",root.size());
    }
  }
  else
    Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

  if (mqttClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
    const int capacity = JSON_OBJECT_SIZE(6);
    StaticJsonDocument<capacity> root;

    root["pHKp"]  = storage.Ph_Kp;    //pH PID coeffcicient Kp
    root["pHKi"]  = storage.Ph_Ki;    //pH PID coeffcicient Ki
    root["pHKd"]  = storage.Ph_Kd;    //pH PID coeffcicient Kd

    root["OrpKp"] = storage.Orp_Kp;    //Orp PID coeffcicient Kp
    root["OrpKi"] = storage.Orp_Ki;    //Orp PID coeffcicient Ki
    root["OrpKd"] = storage.Orp_Kd;    //Orp PID coeffcicient Kd

    if (root.size() < PAYLOAD_BUFFER_LENGTH)
    {
      serializeJson(root,Payload,sizeof(Payload));
      if (mqttClient.publish(PoolTopicSet4, 1, true, Payload) != 0)
      {
        Debug.print(DBG_DEBUG,"Payload: %s - Payload size: %d",Payload,root.size());
      }
      else
      {
        Debug.print(DBG_DEBUG,"Unable to publish the following payload: %s",Payload);
      }
    }
    else
    {
      Debug.print(DBG_ERROR,"MQTT Payload buffer overflow! - Payload size: %d",root.size());
    }
  }
  else
    Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

  if (mqttClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
    const int capacity = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> root;

    root["pHTV"]  = storage.pHTankVol;           //Acid tank nominal volume (Liters)
    root["ChlTV"] = storage.ChlTankVol;          //Chl tank nominal volume (Liters)
    root["pHFR"]  = storage.pHPumpFR;            //Acid pump flow rate (L/hour)
    root["OrpFR"] = storage.ChlPumpFR;           //Chl pump flow rate (L/hour)

    if (root.size() < PAYLOAD_BUFFER_LENGTH)
    {
      serializeJson(root,Payload,sizeof(Payload));
      if (mqttClient.publish(PoolTopicSet5, 1, true, Payload) != 0)
      {
        Debug.print(DBG_DEBUG,"Payload: %s - Payload size: %d",Payload,root.size());
      }
      else
      {
        Debug.print(DBG_DEBUG,"Unable to publish the following payload: %s",Payload);
      }
    }
    else
    {
      Debug.print(DBG_ERROR,"MQTT Payload buffer overflow! - Payload size: %d",root.size());
    }
  }
  else
    Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

  //display remaining RAM space. For debug
  Debug.print(DBG_VERBOSE,"[memCheck]: %db",freeRam());
}

//PublishData loop. Publishes system info/data to MQTT broker every XX secs (30 secs by default)
void PublishDataCallback(Task* me)
{
  //Store the GPIO states in one Byte (more efficient over MQTT)
  EncodeBitMap();

  if (mqttClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
    //Will publish something like {"Tmp":818,"pH":321,"PSI":56,"Orp":583,"FilUpT":8995,"PhUpT":0,"ChlUpT":0}
    const int capacity = JSON_OBJECT_SIZE(7);
    StaticJsonDocument<capacity> root;

    root["TE"]      = storage.TempExternal * 100;        // /!\ x100
    root["Tmp"]     = storage.TempValue * 100;
    root["pH"]      = storage.PhValue * 100;
    root["PSI"]     = storage.PSIValue * 100;
    root["Orp"]     = storage.OrpValue;
    root["PhUpT"]   = PhPump.UpTime / 1000;
    root["ChlUpT"]  = ChlPump.UpTime / 1000;

    /*String tp = "Settings JSON buffer size is: ";
      tp += jsonBuffer.size();
      DEBUG_PRINT(tp);*/

    if (root.size() < PAYLOAD_BUFFER_LENGTH)
    {
      serializeJson(root,Payload,sizeof(Payload));
      if (mqttClient.publish(PoolTopicMeas1, 1, true, Payload) != 0)
      {
        Debug.print(DBG_DEBUG,"Payload: %s - Payload size: %d",Payload,root.size());
      }
      else
      {
        Debug.print(DBG_DEBUG,"Unable to publish the following payload: %s",Payload);
      }
    }
    else
    {
      Debug.print(DBG_ERROR,"MQTT Payload buffer overflow! - Payload size: %d",root.size());
    }
  }
  else
    Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

  //Second MQTT publish to limit size of payload at once
  if (mqttClient.connected())
  {
    //send a JSON to MQTT broker. /!\ Split JSON if longer than 100 bytes
    //Will publish something like {"AcidF":100,"ChlF":100,"IO":11,"IO2":0}
    const int capacity = JSON_OBJECT_SIZE(4);
    StaticJsonDocument<capacity> root;

    root["AcidF"] = storage.AcidFill - PhPump.GetTankUsage();
    root["ChlF"]  = storage.ChlFill - ChlPump.GetTankUsage();
    root["IO"]    = BitMap1;
    root["IO2"]   = BitMap2;

    if (root.size() < PAYLOAD_BUFFER_LENGTH)
    {
      serializeJson(root,Payload,sizeof(Payload));
      if (mqttClient.publish(PoolTopicMeas2, 1, true, Payload) != 0)
      {
        Debug.print(DBG_DEBUG,"Payload: %s - Payload size: %d",Payload,root.size());
      }
      else
      {
        Debug.print(DBG_DEBUG,"Unable to publish the following payload: %s",Payload);
      }
    }
    else
    {
      Debug.print(DBG_ERROR,"MQTT Payload buffer overflow! - Payload size: %d",root.size());
    }
  }
  else
    Debug.print(DBG_ERROR,"Failed to connect to the MQTT broker");

  //display remaining RAM space. For debug
  Debug.print(DBG_VERBOSE,"[memCheck]: %db",freeRam());
}