#include "wificonfig.h"

static const char *TAG = "WiFiConfig";
static char current_ssid[32];
static char current_password[64];
static bool factory_state = false;

// ³õÊ¼»¯ NVS ºÍ Wi-Fi Ä£¿é
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

void load_wifi_credentials(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed (%s), using default credentials", esp_err_to_name(ret));
        strcpy(current_ssid, DEFAULT_ESP_WIFI_SSID);
        strcpy(current_password, DEFAULT_ESP_WIFI_PASS);
        factory_state = true;
        return;
    }

    size_t ssid_len = sizeof(current_ssid);
    size_t pass_len = sizeof(current_password);
    bool ssid_ok = (nvs_get_str(nvs_handle, NVS_KEY_SSID, current_ssid, &ssid_len) == ESP_OK);
    bool pass_ok = (nvs_get_str(nvs_handle, NVS_KEY_PASS, current_password, &pass_len) == ESP_OK);
    nvs_close(nvs_handle);

    if (!ssid_ok) strcpy(current_ssid, DEFAULT_ESP_WIFI_SSID);
    if (!pass_ok) strcpy(current_password, DEFAULT_ESP_WIFI_PASS);
    factory_state = !(ssid_ok && pass_ok);

    ESP_LOGI(TAG, "Loaded WiFi SSID: %s%s",
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
        ESP_LOGI(TAG, "WiFi credentials saved to NVS: SSID=%s", ssid);
        factory_state = false;
    } else {
        ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
    }
    return ret;
}

void update_wifi_config(const char* ssid, const char* password) {
    ESP_LOGI(TAG, "Updating WiFi config to STA mode: SSID=%s", ssid);

    // Stop current AP mode
    esp_wifi_stop();

    // Create default STA netif if not exists
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    if (sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to create default STA netif");
        return;
    }

    // Configure STA
    wifi_config_t sta_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    strlcpy((char *)sta_config.sta.ssid, ssid, sizeof(sta_config.sta.ssid));
    strlcpy((char *)sta_config.sta.password, password, sizeof(sta_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));

    // Start STA
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "STA mode started with SSID: %s", ssid);
}

void wifi_init_softap(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    load_wifi_credentials();

    wifi_config_t ap_config = {
        .ap = {
            .channel = ESP_WIFI_CHANNEL,
            .authmode = strlen(current_password) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN,
            .max_connection = 2,
            .beacon_interval = 100,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };
    strlcpy((char *)ap_config.ap.ssid, current_ssid, sizeof(ap_config.ap.ssid));
    strlcpy((char *)ap_config.ap.password, current_password, sizeof(ap_config.ap.password));
    ap_config.ap.ssid_len = strlen(current_ssid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    // ADDED: Create default AP netif (enables DHCP server)
    esp_netif_create_default_wifi_ap();

    ESP_ERROR_CHECK(esp_wifi_start());

    // ADDED: Log AP IP for verification
    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_AP_IF"), &ip_info) == ESP_OK) {
        ESP_LOGI(TAG, "AP IP: " IPSTR ", Mask: " IPSTR ", GW: " IPSTR,
                 IP2STR(&ip_info.ip), IP2STR(&ip_info.netmask), IP2STR(&ip_info.gw));
    } else {
        ESP_LOGE(TAG, "Failed to get AP IP info");
    }

    ESP_LOGI(TAG, "AP mode started, SSID: %s, password: %s%s",
             current_ssid,
             strlen(current_password) ? "******" : "(open)",
             factory_state ? " [factory]" : "");
}

// Ô¤Áô STA Ä£Ê½³õÊ¼»¯
void wifi_init_sta(void) {
    ESP_LOGI(TAG, "STA mode init (not yet implemented)");
}
