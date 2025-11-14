#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

#include <string.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#define DEFAULT_TARGET_SSID   "FUNLIGHT"  // Fallback
#define DEFAULT_TARGET_PASS   "funlight"
#define NVS_NAMESPACE           "config"
#define NVS_KEY_SSID            "target_ssid"
#define NVS_KEY_PASS            "target_pass"
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT       BIT1
#define MAX_RETRY           5

extern EventGroupHandle_t s_wifi_event_group;

#ifdef __cplusplus
extern "C" {
#endif

void wifi_config_init(void);
void wifi_init_sta(void);  // Client: Only STA mode
void wifi_clear_config(void);
bool wifi_is_factory_mode(void);
void load_wifi_credentials(void);
esp_err_t save_wifi_credentials(const char* ssid, const char* password);  // Optional: For saving target AP

#ifdef __cplusplus
}
#endif

#endif