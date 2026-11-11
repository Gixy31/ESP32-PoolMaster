// Tâche de supervision des connexions WiFi et MQTT
// Catte tâche de fond boucle avec une période de 1s et surveille les connexions
// avec la stratégie de reconnexion suivante :
// - Le gestionnaire d'évènements ne fait que noter ce qui se passe (bits d'état, cause, compteur),
//   aucun appel bloquant ;
// - La réaction se fait par paliers, selon la durée de la coupure :
//   - 0 à 2 min : Disconnect + begin, avec un délai croissant (2/4/8/16 puis 30s), sans interrompre
//                 un DHCP en cours
//   - 2 à 30 min : réinitialisation complète de la radio (WIFI_OF puis WIFI_STA) toutes les 10 min
//   - après 30 min : sauvegarde de l'état puis esp_restart()
//   - IP présente, mais broker injoignable pendant 10 min traité comme un zombie : réinitialisation 
//     de la radio

#include <Arduino.h>                // Arduino framework
#include "Config.h"
#include "PoolMaster.h"

volatile uint8_t  lastReason      = 0;   // dernière cause de déconnexion
volatile uint32_t disconnectCount = 0;   // diagnostic, publiable sur MQTT

// Paramètres de la stratégie de reconnexion
#define DHCP_TIMEOUT_MS        15000UL          // délai laissé au DHCP avant de retenter
#define RETRY_MAX_MS           30000UL          // délai maximal entre deux tentatives
#define RADIO_RESET_AFTER_MS  120000UL          // première réinitialisation radio après 2 min
#define RADIO_RESET_EVERY_MS  600000UL          // puis toutes les 10 min
#define REBOOT_AFTER_MS      1800000UL          // redémarrage après 30 min sans réseau
#define MQTT_RETRY_MS           5000UL          // délai entre tentatives de reconnexions à MQTT
#define MQTT_ZOMBIE_MS        600000UL          // IP présente mais broker injoignable pendant 10 min

bool saveParam(const char*,double);

// Gestionnaire d'événements : il ne fait que noter, aucun appel bloquant
void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      xEventGroupSetBits(netEvents, NET_ASSOCIATED);
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      xEventGroupSetBits(netEvents, NET_ASSOCIATED | NET_HAS_IP);
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      xEventGroupClearBits(netEvents, NET_HAS_IP);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      xEventGroupClearBits(netEvents, NET_ASSOCIATED | NET_HAS_IP);
      lastReason = info.wifi_sta_disconnected.reason;
      disconnectCount++;
      break;
    default:
      break;
  }
}

void wifiStart()
{
  WiFi.setHostname(HOSTNAME);           // avant le passage en mode STA
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);         // NetSupervisor est seul responsable
  WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
}

static void wifiRetry()
{
  WiFi.disconnect(false, false);        // garde la radio allumée et la configuration
  WiFi.begin();
}

static void wifiRadioReset()
{
  Debug.print(DBG_WARNING, "[Net] Radio reset");
  WiFi.disconnect(true, false);         // coupe la radio
  WiFi.mode(WIFI_OFF);
  vTaskDelay(pdMS_TO_TICKS(500));
  wifiStart();
}

void NetSupervisor(void *pvParameters)
{
  uint32_t downSince = 0, lastAttempt = 0, lastRadioReset = 0;
  uint32_t mqttDownSince = 0, lastMqttTry = 0;
  uint8_t  attempts = 0;
  bool     wasUp = false;

  esp_task_wdt_add(nullptr);            // la boucle tourne chaque seconde

  for (;;) {
    esp_task_wdt_reset();
    const uint32_t now  = millis();
    const EventBits_t b = xEventGroupGetBits(netEvents);

    if (b & NET_HAS_IP) {
      // ------------------- Réseau disponible -------------------
      if (!wasUp) {
        Debug.print(DBG_INFO, "[Net] Up (down %lus, reason %u, count %lu)",
                    downSince ? (now - downSince) / 1000 : 0, lastReason, disconnectCount);
        wasUp = true;
      }
      downSince = 0; attempts = 0;

      // Connexion MQTT gérée ici
      if (!mqttClient.connected()) {
        if (!mqttDownSince) mqttDownSince = now;
        if (now - lastMqttTry >= MQTT_RETRY_MS) {
          lastMqttTry = now;
          mqttClient.connect();
        }
        // Zombie : IP valide mais broker injoignable trop longtemps
        if (now - mqttDownSince >= MQTT_ZOMBIE_MS) {
          wifiRadioReset();
          mqttDownSince = now;
        }
      } else {
        mqttDownSince = 0;
      }
    } else {
      // ------------------- Réseau absent -------------------
      if (wasUp) {
        Debug.print(DBG_WARNING, "[Net] Down, reason %u", lastReason);
        wasUp = false;
      }
      if (!downSince) { downSince = now; lastAttempt = 0; }
      const uint32_t down = now - downSince;

      // Ne pas interrompre un DHCP en cours
      const bool dhcpPending = (b & NET_ASSOCIATED) && (now - lastAttempt < DHCP_TIMEOUT_MS);
      const uint32_t backoff = min(2000UL << attempts, RETRY_MAX_MS);

      if (!dhcpPending && (lastAttempt == 0 || now - lastAttempt >= backoff)) {
        lastAttempt = now;
        if (down >= RADIO_RESET_AFTER_MS && now - lastRadioReset >= RADIO_RESET_EVERY_MS) {
          lastRadioReset = now;
          wifiRadioReset();
        } else {
          wifiRetry();
        }
        if (attempts < 4) attempts++;
      }

      if (down >= REBOOT_AFTER_MS) {
        Debug.print(DBG_ERROR, "[Net] No network for 30 min, restarting");
        saveParam("AcidFill", PhPump.GetTankFill());
        saveParam("ChlFill",  ChlPump.GetTankFill());
        esp_restart();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}