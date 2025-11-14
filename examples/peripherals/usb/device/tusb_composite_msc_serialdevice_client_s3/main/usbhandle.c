#include "usbhandle.h"
#include "tcp_client.h"  // For shared tx_ring, app_queue, TAG

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
    .idProduct = 0x4001,  // Different product ID for client
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
    "ESP32-S3 CDC Client",  // Updated product string
    "1234567891",  // Different serial
    "CDC Interface"
};

static const char *TAG = "usbhandle";
static uint8_t cdc_rx_buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];

void app_queue_init() {
    size_t free_heap = esp_get_free_heap_size();
    ESP_LOGI(TAG, "Free heap before queue creation: %zu bytes", free_heap);
    app_queue = xQueueCreate(QUEUE_SIZE, sizeof(app_message_t));
    if (!app_queue) {
        ESP_LOGE(TAG, "Failed to create app_queue, free heap: %zu bytes", free_heap);
        esp_restart();
    }
    ESP_LOGI(TAG, "app_queue created successfully");
}

void tinyusb_cdc_line_coding_changed_callback(int itf, cdcacm_event_t *event) {
    cdcacm_event_line_coding_changed_data_t *coding = &event->line_coding_changed_data;
    char message[128];
    size_t msg_len = snprintf(message, sizeof(message), "BAUD=%lu,STOPBIT=%d,PARITY=%d,DATABIT=%d\r\n",
             coding->p_line_coding->bit_rate, coding->p_line_coding->stop_bits,
             coding->p_line_coding->parity, coding->p_line_coding->data_bits);
    ESP_LOGI(TAG, "Baud rate update: %s", message);

    // Send to TCP (client version)
    send_control_to_tcp(PROTO_TYPE_CMD, (const uint8_t *)message, msg_len);
}

static uint32_t total_received = 0;
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event) {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    uint8_t *rx_ptr = buf;
    size_t remaining = tud_cdc_available();
    size_t rx_size = 0;
    while (remaining > 0) {
        size_t chunk = tud_cdc_read(rx_ptr, (remaining < sizeof(buf) - rx_size ? remaining : sizeof(buf) - rx_size));
        if (chunk == 0) break;
        rx_size += chunk;
        rx_ptr += chunk;
        remaining -= chunk;
    }
    if (rx_size > 0) {
        app_message_t msg = {
            .buf_len = rx_size,
            .itf = itf,
        };
        memcpy(msg.buf, buf, rx_size);

        total_received += rx_size;
        if (total_received % 5000 == 0) ESP_LOGI(TAG, "Total USB received: %u bytes", total_received);
        // Enqueue
        if (xQueueSend(app_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "app_queue full: dropped %zu RX bytes", rx_size);
        } else {
            // Notify usb_to_tcp_task
            TaskHandle_t usb_task_handle = xTaskGetHandle("usb_to_tcp");
            if (usb_task_handle != NULL) {
                BaseType_t higher_priority = pdFALSE;
                vTaskNotifyGiveFromISR(usb_task_handle, &higher_priority);
                portYIELD_FROM_ISR(higher_priority);
            }
        }
    }
}


void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event) {
    if (!event->line_state_changed_data.dtr || !event->line_state_changed_data.rts) {
        tud_cdc_n_write_clear(TINYUSB_CDC_ACM_0);
        tud_cdc_n_read_flush(TINYUSB_CDC_ACM_0);
    }
}

esp_err_t init_usb(void) {
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
        ESP_LOGE(TAG, "TinyUSB CDC ACM init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    app_queue_init();
    ESP_LOGI(TAG, "USB CDC initialized");
    return ESP_OK;
}
