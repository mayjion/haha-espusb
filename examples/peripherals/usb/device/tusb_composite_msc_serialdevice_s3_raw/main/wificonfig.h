#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <string.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#define DEFAULT_ESP_WIFI_SSID   "FUNLIGHT"
#define DEFAULT_ESP_WIFI_PASS   "funlight"
#define ESP_WIFI_CHANNEL        0
#define NVS_NAMESPACE           "config"
#define NVS_KEY_SSID            "wifi_ssid"
#define NVS_KEY_PASS            "wifi_pass"

#ifdef __cplusplus
extern "C" {
#endif

void wifi_config_init(void);
void wifi_init_softap(void);
void wifi_init_sta(void);
void wifi_clear_config(void);
bool wifi_is_factory_mode(void);
void load_wifi_credentials(void);
esp_err_t save_wifi_credentials(const char* ssid, const char* password);
void update_wifi_config(const char* ssid, const char* password);

#ifdef __cplusplus
}
#endif

#endif
