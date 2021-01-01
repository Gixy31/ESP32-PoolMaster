/*
* Task: monitor the WiFi connection and keep it alive!
* 
* When a WiFi connection is established, this task will check it every 10 seconds 
* to make sure it's still alive.
* 
* If not, a reconnect is attempted. If this fails to finish within the timeout,
* the ESP32 will wait for it to recover and try again.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>

WiFiMulti wlan;

#define WIFI_TIMEOUT_MS 20000      // 20 second WiFi connection timeout
#define WIFI_RECOVER_TIME_MS 30000 // Wait 30 seconds after a failed connection attempt
#define LED 2

void keepWiFiAlive(void *parameter)
{
  pinMode(LED, OUTPUT);
  for (;;)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    else
    {
      digitalWrite(LED, LOW);
      Serial.println("[WIFI] Connecting");
      WiFi.mode(WIFI_STA);
      WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
      esp_err_t ret = tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA ,"PoolMaster");
      if(ret != ESP_OK ) Serial.println("Failed to set hostname");      
      wlan.addAP("SSID1","PWD1");
      wlan.addAP("SSID2","PWD2");
      wlan.addAP("SSID3","PWD3");
      unsigned long startAttemptTime = millis();

      // Keep looping while we're not connected and haven't reached the timeout
      // while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
      while (wlan.run() != WL_CONNECTED && millis() - startAttemptTime < WIFI_TIMEOUT_MS)
      {
        Serial.print(".");
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }

      // When we couldn't make a WiFi connection (or the timeout expired)
      // sleep for a while and then retry.
      if (WiFi.status() != WL_CONNECTED)
      {
        Serial.println("[WIFI] FAILED");
        vTaskDelay(WIFI_RECOVER_TIME_MS / portTICK_PERIOD_MS);
      }
      else
      {
        Serial.print("[WIFI] Connected to: ");
        Serial.println(WiFi.SSID());
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        Serial.print("Hostname: ");
        Serial.println(WiFi.getHostname());
        digitalWrite(LED, HIGH);
      }
    }
  }
}