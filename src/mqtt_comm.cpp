// MQTT related functions for PoolMaster, including WiFi functions
// Use JSON version 6

#undef __STRICT_ANSI__
#include <Arduino.h>
#include "Config.h"
#include "PoolMaster.h"

AsyncMqttClient mqttClient;

bool MQTTConnection = false;                  // Status of connection to broker
static TimerHandle_t mqttReconnectTimer;      // Reconnect timer for MQTT
static TimerHandle_t wifiReconnectTimer;      // Reconnect timer for WiFi

#ifdef MQTT_LOGIN
 static const char* MqttServerClientID = MQTT_SERVER_ID;            
 static const char* MqttServerLogin    = MQTT_SERVER_LOGIN;                
 static const char* MqttServerPwd      = MQTT_SERVER_PWD;            
#endif

static const char* PoolTopicAPI       = POOLTOPIC"API";
static const char* PoolTopicStatus    = POOLTOPIC"Status";
static const char* PoolTopicError     = POOLTOPIC"Err";

// Functions prototypes
void initTimers(void);
void mqttInit(void);
void mqttErrorPublish(const char* );
void connectToWiFi(void);
void reconnectToWiFi();
void connectToMqtt(void);
void WiFiEvent(WiFiEvent_t );
void onMqttConnect(bool);
void onMqttDisconnect(AsyncMqttClientDisconnectReason);
void onMqttSubscribe(uint16_t, uint8_t);
void onMqttUnSubscribe(uint16_t);
void onMqttMessage(char* , char* , AsyncMqttClientMessageProperties , size_t , size_t , size_t );
void onMqttPublish(uint16_t);
void UpdateWiFi(bool);
int  freeRam(void);

void initTimers() {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(reconnectToWiFi));
}

void mqttInit() {
  //Init Async MQTT
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnSubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setWill(PoolTopicStatus,1,true,"{\"PoolMaster Online\":0}");
  mqttClient.setServer(MQTT_SERVER_IP,MQTT_SERVER_PORT);
#ifdef MQTT_LOGIN  
  mqttClient.setCredentials(MqttServerLogin,MqttServerPwd);
  mqttClient.setClientId(MqttServerClientID);
#endif
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

void connectToWiFi(){
  Debug.print(DBG_INFO,"[WiFi] Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(HOSTNAME); 
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
}

void reconnectToWiFi(){
  if(WiFi.status() != WL_CONNECTED){
    Debug.print(DBG_INFO,"[WiFi] Reconnecting to WiFi...");
    WiFi.reconnect();
  } else Debug.print(DBG_INFO,"[WiFi] Spurious disconnect event ignored");    
}

void connectToMqtt(){
  mqttClient.connect();
}

void WiFiEvent(WiFiEvent_t event){
  switch(event){
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      xTimerStop(wifiReconnectTimer,0);
      Debug.print(DBG_INFO,"[WiFi] Connected to: %s",WiFi.SSID().c_str());
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      xTimerStop(wifiReconnectTimer,0);
      Debug.print(DBG_INFO,"[WiFi] IP address: %s",WiFi.localIP().toString().c_str());
      Debug.print(DBG_INFO,"[WiFi] Hostname: %s",WiFi.getHostname());
      Debug.print(DBG_INFO,"[WiFi] Connecting to MQTT...");
      UpdateWiFi(true);
      connectToMqtt();
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Debug.print(DBG_WARNING,"[WiFi] Connection lost");
      xTimerStop(mqttReconnectTimer,0);
      xTimerStart(wifiReconnectTimer,0);
      UpdateWiFi(false);
      break;    
    default:
      break;  
  }
}

// Once connected to MQTT broker, subscribe to the PoolTopicAPI topic in order to receive future commands
// then publish the "online" message on the "status" topic. If Ethernet connection is ever lost
// "status" will switch to "offline". Very useful to check that the system is alive and functional
void onMqttConnect(bool sessionPresent){
  Debug.print(DBG_INFO,"Connected to MQTT, present session: %d",sessionPresent);
  mqttClient.subscribe(PoolTopicAPI,2);
  mqttClient.publish(PoolTopicStatus,1,true,"{\"PoolMaster Online\":1}");
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
    char Command[100] = "";

    for (uint8_t i=0 ; i<len ; i++){
      Command[i] = payload[i];
    }
    if (xQueueSendToBack(queueIn, &Command, 0) == pdPASS)
    {
      Debug.print(DBG_INFO,"Command added to queue: %s",Command);
    }
    else
    {
      Debug.print(DBG_ERROR,"Queue full, command: %s not added", Command);
    }
    Debug.print(DBG_DEBUG,"FreeRam: %d Queued messages: %d",freeRam(),uxQueueMessagesWaiting(queueIn));
  }
}