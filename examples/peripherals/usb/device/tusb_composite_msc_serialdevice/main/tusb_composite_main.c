#include "tusb_composite_main.h"

static const char *TAG = "example_main";
static uint8_t cdc_rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
static QueueHandle_t app_queue;
static int client_sock = -1;
static bool client_active = false;
static char cached_log_prefix[64] = "[00:00:00.000][device 0]";
static char current_ssid[32] = DEFAULT_ESP_WIFI_SSID;
static char current_password[64] = DEFAULT_ESP_WIFI_PASS;
static bool factory_state = true;

// Device descriptor
static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0xCafe,
    .idProduct = 0x4000,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

// Configuration descriptor (single CDC)
static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN), TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),
    TUD_CDC_DESCRIPTOR(0, 4, 0x81, 8, 0x02, 0x82, 64),
};

// String descriptors
static const char *string_desc_arr[] = {
    (const char[]){0x09, 0x04}, // Language: English (US)
    "Manufacturer",
    "ESP32-S2 CDC Device",
    "1234567890",
    "CDC Interface"
};

static void update_log_prefix(const char *client_ip) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tm_info = gmtime(&tv.tv_sec);
    const char *last_octet = strrchr(client_ip, '.') ? strrchr(client_ip, '.') + 1 : "0";
    snprintf(cached_log_prefix, sizeof(cached_log_prefix), "[%02d:%02d:%02d.%03d][device %s]",
             tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec, (int)(tv.tv_usec / 1000), last_octet);
}

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(itf, cdc_rx_buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK && rx_size > 0) {
        app_message_t msg = { .buf_len = rx_size, .itf = itf };
        memcpy(msg.buf, cdc_rx_buf, rx_size);
        if (xQueueSend(app_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Queue full, packet dropped");
        }
    }
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event) {
    if (!event->line_state_changed_data.dtr || !event->line_state_changed_data.rts) {
        tud_cdc_n_write_clear(TINYUSB_CDC_ACM_0);
        tud_cdc_n_read_flush(TINYUSB_CDC_ACM_0);
    }
}

static void send_to_client(const uint8_t *data, size_t len, const char *log_prefix) {
    if (!client_active || client_sock < 0) return;
    int sent = send(client_sock, data, len, 0);
    if (sent < 0) {
        ESP_LOGW(TAG, "%s TCP send failed, closing socket", log_prefix);
        close(client_sock);
        client_sock = -1;
        client_active = false;
        gpio_set_level(LED_GPIO, 0);
    }
}


void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event) {
    cdcacm_event_line_coding_changed_data_t *coding = &event->line_coding_changed_data;
    char message[128];
    snprintf(message, sizeof(message), "BAUD=%lu,STOPBIT=%d,PARITY=%d,DATABIT=%d\r\n",
             coding->p_line_coding->bit_rate, coding->p_line_coding->stop_bits,
             coding->p_line_coding->parity, coding->p_line_coding->data_bits);
    send_to_client((uint8_t *)message, strlen(message), cached_log_prefix);
    ESP_LOGI(TAG, "%s Sent baud rate update: %s", cached_log_prefix, message);
}

static esp_err_t load_wifi_credentials(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NVS open failed: %s, using default credentials", esp_err_to_name(ret));
        return ret;
    }
    size_t ssid_len = sizeof(current_ssid);
    ret = nvs_get_str(nvs_handle, NVS_KEY_SSID, current_ssid, &ssid_len);
    if (ret != ESP_OK) {
        strlcpy(current_ssid, DEFAULT_ESP_WIFI_SSID, sizeof(current_ssid));
        factory_state = true;
    } else {
        factory_state = false;
    }
    size_t pass_len = sizeof(current_password);
    ret = nvs_get_str(nvs_handle, NVS_KEY_PASS, current_password, &pass_len);
    if (ret != ESP_OK) {
        strlcpy(current_password, DEFAULT_ESP_WIFI_PASS, sizeof(current_password));
        factory_state = true;
    } else {
        factory_state = factory_state && false;
    }
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Loaded WiFi credentials: SSID=%s, Password=%s, Factory=%d",
             current_ssid, factory_state ? "default" : current_password, factory_state);
    return ESP_OK;
}

static esp_err_t save_wifi_credentials(const char *ssid, const char *password) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_set_str(nvs_handle, NVS_KEY_SSID, ssid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS set SSID failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    ret = nvs_set_str(nvs_handle, NVS_KEY_PASS, password);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS set password failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Saved WiFi credentials: SSID=%s, Password=%s", ssid, password);
    return ESP_OK;
}

static esp_err_t clear_nvs_config(void) {
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = nvs_erase_key(nvs_handle, NVS_KEY_SSID);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS erase SSID failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    ret = nvs_erase_key(nvs_handle, NVS_KEY_PASS);
    if (ret != ESP_OK && ret != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "NVS erase password failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    ret = nvs_commit(nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS commit failed: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "NVS config cleared, reverting to factory settings");
    return ESP_OK;
}

static void update_wifi_config(const char *ssid, const char *password) {
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(ssid),
            .channel = ESP_WIFI_CHANNEL,
            .authmode = strlen(password) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .max_connection = 1,
            .beacon_interval = 100,
            .pmf_cfg = { .capable = true, .required = false },
            .ftm_responder = false
        },
    };
    strlcpy((char *)wifi_config.ap.ssid, ssid, sizeof(wifi_config.ap.ssid));
    strlcpy((char *)wifi_config.ap.password, password, sizeof(wifi_config.ap.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_LOGI(TAG, "Updated WiFi config: SSID=%s, Password=%s", ssid, password);
}

static void button_task(void *pvParameters) {
    TickType_t last_press_time = 0;
    bool last_state = true;
    while (1) {
        bool current_state = gpio_get_level(BUTTON_GPIO);
        if (last_state && !current_state) {
            last_press_time = xTaskGetTickCount();
        } else if (!last_state && current_state) {
            TickType_t press_duration = xTaskGetTickCount() - last_press_time;
            ESP_LOGI(TAG, "BUTTON PRESSED: %d, ticks:%d", press_duration,pdMS_TO_TICKS(press_duration));
            if (press_duration < pdMS_TO_TICKS(BUTTON_CLICK_THRESHOLD_MS) && factory_state && client_active) {
                uint8_t mac[6];
                char new_ssid[32];
                char new_password[64];
                char mac_str[13];
                esp_wifi_get_mac(WIFI_IF_AP, mac);
                snprintf(mac_str, sizeof(mac_str), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                snprintf(new_ssid, sizeof(new_ssid), "FUNLIGHT-%s", &mac_str[8]);
                snprintf(new_password, sizeof(new_password), "funlight-%s", &mac_str[8]);
                update_log_prefix("0.0.0.0");
                char message[128];
                snprintf(message, sizeof(message), "WIFI=%s,%s\r\n", new_ssid, new_password);
                send_to_client((uint8_t *)message, strlen(message), cached_log_prefix);
                ESP_LOGI(TAG, "%s Sent WiFi credentials to client: %s", cached_log_prefix, message);
                vTaskDelay(pdMS_TO_TICKS(2000));  // Ensure client receives notification
                esp_err_t ret = save_wifi_credentials(new_ssid, new_password);
                if (ret == ESP_OK) {
                    update_wifi_config(new_ssid, new_password);
                    strlcpy(current_ssid, new_ssid, sizeof(current_ssid));
                    strlcpy(current_password, new_password, sizeof(current_password));
                    factory_state = false;
                    ESP_LOGI(TAG, "%s WiFi credentials updated, restarting", cached_log_prefix);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else {
                    ESP_LOGE(TAG, "%s Failed to save WiFi credentials: %s", cached_log_prefix, esp_err_to_name(ret));
                }
            }
        } else if (!last_state && !current_state) {
            TickType_t press_duration = xTaskGetTickCount() - last_press_time;
            ESP_LOGI(TAG, "BUTTON PRESSED11: %d, ticks:%d", press_duration,pdMS_TO_TICKS(press_duration));
            if (press_duration >= pdMS_TO_TICKS(BUTTON_LONG_PRESS_MS)) {
                esp_err_t ret = clear_nvs_config();
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "%s Factory reset successful, restarting", cached_log_prefix);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else {
                    ESP_LOGE(TAG, "%s Factory reset failed: %s", cached_log_prefix, esp_err_to_name(ret));
                }
            }
        }
        last_state = current_state;
        vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
    }
}


static void wifi_init_softap(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    load_wifi_credentials();
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(current_ssid),
            .channel = ESP_WIFI_CHANNEL,
            .authmode = strlen(current_password) == 0 ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
            .max_connection = 1,
            .beacon_interval = 100,
            .pmf_cfg = { .capable = true, .required = false },
            .ftm_responder = false
        },
    };
    strlcpy((char *)wifi_config.ap.ssid, current_ssid, sizeof(wifi_config.ap.ssid));
    strlcpy((char *)wifi_config.ap.password, current_password, sizeof(wifi_config.ap.password));
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();
    ESP_LOGI(TAG, "WiFi AP started with SSID: %s", current_ssid);
}

static void usb_to_tcp_task(void *pvParameters) {
    app_message_t msg;
    while (1) {
        if (xQueueReceive(app_queue, &msg, portMAX_DELAY)) {
            update_log_prefix("0.0.0.0");
            send_to_client(msg.buf, msg.buf_len, cached_log_prefix);
        }
    }
}

static void tcp_server_task(void *pvParameters) {
    struct sockaddr_in dest_addr = { .sin_addr.s_addr = htonl(INADDR_ANY), .sin_family = AF_INET, .sin_port = htons(PORT) };
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    bind(listen_sock, (struct sockaddr*)&dest_addr, sizeof(dest_addr));
    listen(listen_sock, 1);

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr*)&source_addr, &addr_len);
        if (sock < 0) continue;

        if (client_active) {
            close(sock);
            continue;
        }

        int nodelay = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
        int sndbuf = 4096;
        setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

        client_sock = sock;
        client_active = true;
        gpio_set_level(LED_GPIO, 1);
        ESP_LOGI(TAG, "Client connected, LED on");

        uint8_t rx_buffer[UART_BUF_SIZE];
        while (client_active) {
            int len = recv(client_sock, rx_buffer, UART_BUF_SIZE, 0);
            if (len <= 0) break;
            size_t queued = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, rx_buffer, len);
            for (int retry = 0; retry < 5; retry++) {
                if (tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(CDC_FLUSH_TIMEOUT_MS)) == ESP_OK && queued == len) break;
            }
        }

        close(client_sock);
        client_sock = -1;
        client_active = false;
        gpio_set_level(LED_GPIO, 0);
        ESP_LOGI(TAG, "Client disconnected, LED off");
    }
}

static void button_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static esp_err_t init_system(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(LED_GPIO, 0);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    wifi_init_softap();
    button_init();
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("tusb", ESP_LOG_DEBUG);
    size_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Free heap before queue creation: %zu bytes", free_heap);
    app_queue = xQueueCreate(QUEUE_SIZE, sizeof(app_message_t));
    if (!app_queue) {
        ESP_LOGE(TAG, "Failed to create app_queue, free heap: %zu bytes", free_heap);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "app_queue created successfully");
    return ESP_OK;
}

static esp_err_t init_usb(void) {
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration
    };
    esp_err_t ret = tinyusb_driver_install(&tusb_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TinyUSB install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = CONFIG_TINYUSB_CDC_RX_BUFSIZE,
        .callback_rx = &tinyusb_cdc_rx_callback,
        .callback_line_state_changed = &tinyusb_cdc_line_state_changed_callback,
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback,
    };
    ret = tusb_cdc_acm_init(&acm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CDC ACM init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "USB CDC initialized successfully");
    return ESP_OK;
}

static void usb_device_task(void *param) {
    vTaskDelay(pdMS_TO_TICKS(100));
    while (1) {
        tud_task();
        vTaskDelay(1);
    }
}

void app_main(void) {
    esp_err_t ret = init_system();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "System init failed, restarting");
        esp_restart();
    }
    ret = init_usb();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB init failed, restarting");
        esp_restart();
    }

    xTaskCreate(usb_device_task, "usb_dev", 8192, NULL, 10, NULL);
    xTaskCreate(tcp_server_task, "tcp_server", 3072, NULL, 8, NULL);
    xTaskCreate(usb_to_tcp_task, "usb_to_tcp", 3072, NULL, 9, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "Tasks created, app_main complete");
}