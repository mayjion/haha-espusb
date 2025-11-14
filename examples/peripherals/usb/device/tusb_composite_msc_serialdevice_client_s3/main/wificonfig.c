#include "wificonfig.h"
#include <stdlib.h>  // For malloc/free
#include <limits.h>  // For INT8_MIN

static const char *TAG = "WiFiConfig";
static char current_ssid[32];
static char current_password[64];
static bool factory_state = false;
EventGroupHandle_t s_wifi_event_group = NULL;
static int s_retry_num = 0;

// WiFi event handler
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "WiFi disconnected, retry %d/%d", s_retry_num, MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGW(TAG, "WiFi connect failed after %d retries", MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initialize NVS and Wi-Fi module
void wifi_config_init(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    ESP_LOGI(TAG, "NVS initialized successfully");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI(TAG, "Network interface and event loop ready");
}

bool wifi_is_factory_mode(void) {
    return factory_state;
}

static esp_err_t clear_nvs_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    nvs_erase_key(nvs_handle, NVS_KEY_SSID);
    nvs_erase_key(nvs_handle, NVS_KEY_PASS);
    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGW(TAG, "NVS WiFi config cleared (factory reset)");
    } else {
        ESP_LOGE(TAG, "Failed to clear NVS WiFi config: %s", esp_err_to_name(ret));
    }
    return ret;
}

void wifi_clear_config(void) {
    clear_nvs_config();
}

static esp_err_t discover_funlight_ap(char* ssid_out, char* pass_out) {
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
    };

    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scan start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint16_t ap_count = 0;
    ret = esp_wifi_scan_get_ap_num(&ap_count);
    if (ret != ESP_OK || ap_count == 0) {
        ESP_LOGW(TAG, "No APs found");
        return ESP_ERR_NOT_FOUND;
    }

    wifi_ap_record_t *ap_list = (wifi_ap_record_t *) malloc(ap_count * sizeof(wifi_ap_record_t));
    if (ap_list == NULL) {
        ESP_LOGE(TAG, "Malloc failed for AP list");
        return ESP_ERR_NO_MEM;
    }

    ret = esp_wifi_scan_get_ap_records(&ap_count, ap_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Scan get records failed: %s", esp_err_to_name(ret));
        free(ap_list);
        return ret;
    }

    wifi_ap_record_t best_ap = {0};
    int8_t best_rssi = INT8_MIN;
    char best_suffix[4] = {0};

    for (int i = 0; i < ap_count; i++) {
        const char* ap_ssid = (const char*) ap_list[i].ssid;
        size_t ssid_len = strlen(ap_ssid);
        ESP_LOGI(TAG, "Found AP: '%s' (len=%zu) auth=%d rssi=%d", ap_ssid, ssid_len, ap_list[i].authmode, ap_list[i].rssi);

        bool is_funlight = false;
        char suffix[4] = {0};

        if (ssid_len == 8 && strncmp(ap_ssid, "FUNLIGHT", 8) == 0) {
            is_funlight = true;
        } else if (ssid_len >= 11 && ssid_len <= 32 && strncmp(ap_ssid, "FUNLIGHT-", 9) == 0) {
            strncpy(suffix, ap_ssid + 9, 3);
            is_funlight = true;
        }

        if (is_funlight && ap_list[i].authmode == WIFI_AUTH_WPA2_PSK && ap_list[i].rssi > best_rssi) {
            best_rssi = ap_list[i].rssi;
            best_ap = ap_list[i];
            strncpy(best_suffix, suffix, sizeof(best_suffix));
        }
    }

    free(ap_list);

    if (best_rssi != INT8_MIN) {
        strncpy(ssid_out, (const char*)best_ap.ssid, sizeof(current_ssid) - 1);
        ssid_out[sizeof(current_ssid) - 1] = '\0';
        if (strlen(best_suffix) == 0) {
            strncpy(pass_out, "funlight", sizeof(current_password));
        } else {
            snprintf(pass_out, sizeof(current_password), "funlight-%s", best_suffix);
        }
        ESP_LOGI(TAG, "Discovered FUNLIGHT AP: %s (RSSI %d dBm)", ssid_out, best_rssi);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "No suitable FUNLIGHT AP found");
    return ESP_ERR_NOT_FOUND;
}

void load_wifi_credentials(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed (%s), using default credentials", esp_err_to_name(ret));
        strcpy(current_ssid, DEFAULT_TARGET_SSID);
        strcpy(current_password, DEFAULT_TARGET_PASS);
        factory_state = true;
        return;
    }

    size_t ssid_len = sizeof(current_ssid);
    size_t pass_len = sizeof(current_password);
    bool ssid_ok = (nvs_get_str(nvs_handle, NVS_KEY_SSID, current_ssid, &ssid_len) == ESP_OK);
    bool pass_ok = (nvs_get_str(nvs_handle, NVS_KEY_PASS, current_password, &pass_len) == ESP_OK);
    nvs_close(nvs_handle);

    if (!ssid_ok) strcpy(current_ssid, DEFAULT_TARGET_SSID);
    if (!pass_ok) strcpy(current_password, DEFAULT_TARGET_PASS);
    factory_state = !(ssid_ok && pass_ok);

    ESP_LOGI(TAG, "Loaded target WiFi SSID: %s%s",
             current_ssid,
             factory_state ? " (factory default)" : "");
}

esp_err_t save_wifi_credentials(const char* ssid, const char* password) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_set_str(nvs_handle, NVS_KEY_SSID, ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set SSID: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_set_str(nvs_handle, NVS_KEY_PASS, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set password: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }

    ret = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Target WiFi credentials saved to NVS: SSID=%s", ssid);
        factory_state = false;
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }
    return ret;
}

void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    load_wifi_credentials();

    esp_netif_create_default_wifi_sta();

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    if (factory_state) {
        // Temporary start for scanning (open mode)
        wifi_config_t temp_config = {0};
        temp_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &temp_config));
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGI(TAG, "Temporary WiFi start for AP discovery");

        vTaskDelay(pdMS_TO_TICKS(2000));  // Longer delay for scan

        esp_err_t discover_ret = discover_funlight_ap(current_ssid, current_password);
        ESP_ERROR_CHECK(esp_wifi_stop());
        vTaskDelay(pdMS_TO_TICKS(500));

        if (discover_ret != ESP_OK) {
            ESP_LOGE(TAG, "Discovery failed, falling back to default");
            strncpy(current_ssid, DEFAULT_TARGET_SSID, sizeof(current_ssid));
            strncpy(current_password, DEFAULT_TARGET_PASS, sizeof(current_password));
        }
    }

    // Set final configuration
    wifi_config_t sta_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
        },
    };
    strlcpy((char *)sta_config.sta.ssid, current_ssid, sizeof(sta_config.sta.ssid));
    strlcpy((char *)sta_config.sta.password, current_password, sizeof(sta_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA mode started, connecting to SSID: %s", current_ssid);
}