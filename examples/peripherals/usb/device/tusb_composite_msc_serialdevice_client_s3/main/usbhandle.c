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

    // 发送格式化协议帧（type=CMD）
    send_control_to_tcp(PROTO_TYPE_CMD, (const uint8_t *)message, msg_len);  // 注意：snprintf 返回长度，不含 \0
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
        // 入队
        if (xQueueSend(app_queue, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "app_queue full: dropped %zu RX bytes", rx_size);
        } else {
            // ADDED: Notify usb_to_tcp_task (从 ISR 安全)
            TaskHandle_t usb_task_handle = NULL;  // 静态或全局获取
            if (usb_task_handle == NULL) {
                usb_task_handle = xTaskGetHandle("usb_to_tcp");  // 任务名匹配 xTaskCreate
            }
            BaseType_t higher_priority = pdFALSE;
            vTaskNotifyGiveFromISR(usb_task_handle, &higher_priority);
            portYIELD_FROM_ISR(higher_priority);
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
        .callback_line_coding_changed = &tinyusb_cdc_line_coding_changed_callback,  // FIXED: Signature now matches
    };
    ret = tusb_cdc_acm_init(&acm_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CDC ACM init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    app_queue_init();  // Added: Initialize shared queue
    ESP_LOGI(TAG, "USB CDC initialized successfully");
    return ESP_OK;
}

void usb_to_tcp_task(void *pvParameters) {
    esp_err_t wdt_err = esp_task_wdt_add(NULL);
    if (wdt_err != ESP_OK && wdt_err != ESP_ERR_NOT_FOUND) {
        ESP_LOGW(TAG, "TWDT add failed: %s", esp_err_to_name(wdt_err));
    }

    app_message_t msg;
    static uint32_t discard_count = 0;
    static uint32_t total_discarded_bytes = 0;
    uint32_t notified_value = 0;

    while (1) {
        // ADDED: Wait for notify or short timeout (1ms for low latency)
        if (xTaskNotifyWait(0, ULONG_MAX, &notified_value, pdMS_TO_TICKS(5)) == pdTRUE || uxQueueMessagesWaiting(app_queue) > 0) {
            esp_task_wdt_reset();  // Reset on activity
            while (xQueueReceive(app_queue, &msg, 0) == pdTRUE) {  // Drain all pending (non-blocking)
                if (msg.buf_len > 0) {
                    ESP_LOGI(TAG, "USB RX: %u bytes", msg.buf_len);
                    // ESP_LOG_BUFFER_HEX(TAG, msg.buf, msg.buf_len);  // ADDED: Dump USB raw 数据确认 (before lock)
                    // Defensive: Zero tail of msg.buf to prevent queue garbage propagation
                    memset(msg.buf + msg.buf_len, 0, sizeof(msg.buf) - msg.buf_len);
                }

                xSemaphoreTake(client_mutex, portMAX_DELAY);
                if (!active_client.active) {
                    xSemaphoreGive(client_mutex);
                    discard_count++;
                    total_discarded_bytes += msg.buf_len;
                    // Rate-limit: Every 50th with total (verifies no loss)
                    if (discard_count % 50 == 0) {
                        ESP_LOGI(TAG, "No TCP client: Discarding %zu USB bytes (event %u, total discarded: %u bytes)", 
                                 msg.buf_len, discard_count, total_discarded_bytes);
                    }
                    continue;
                }
                // Now hold mutex: protect ring write
                size_t to_write = msg.buf_len;
                size_t free_space = TX_BUFFER_SIZE - tx_ring.len;
                if (to_write > free_space) {
                    to_write = free_space;
                    ESP_LOGI(TAG, "TX ring full (%zu/%zu), truncating to %zu bytes (dropped %zu)",
                             tx_ring.len, TX_BUFFER_SIZE, to_write, msg.buf_len - to_write);
                    if (to_write == 0) {
                        xSemaphoreGive(client_mutex);
                        continue;
                    }
                }
                if (to_write > 0) {
                    size_t pos = tx_ring.head;
                    for (size_t i = 0; i < to_write; ++i) {
                        tx_ring.buf[pos] = msg.buf[i];
                        pos = (pos + 1) % tx_ring.size;  // 修改：使用 tx_ring.size
                    }
                    tx_ring.head = pos;
                    tx_ring.len += to_write;
                    ESP_LOGD(TAG, "Queued %zu USB bytes (ring: %zu/%zu)", to_write, tx_ring.len, tx_ring.size);

                    // 添加：写后验证（无环绕时）
                    if (tx_ring.tail == 0 && to_write >= 3) {
                        ESP_LOGI(TAG, "Ring write check: buf[0]=0x%02X, buf[1]=0x%02X, buf[2]=0x%02X", 
                                 tx_ring.buf[0], tx_ring.buf[1], tx_ring.buf[2]);
                    }
                }
                xSemaphoreGive(client_mutex);

                if (tx_ring.len > (TX_BUFFER_SIZE * 0.8)) {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
            }
        } else {
            esp_task_wdt_reset();  // Idle reset
        }
    }
}
void parse_and_usb_task(void *pvParameters) {
    esp_task_wdt_add(NULL);
    uint8_t parse_buf[FRAME_BUFFER_SIZE];
    uint8_t payload[FRAME_BUFFER_SIZE];
    const TickType_t parse_delay = pdMS_TO_TICKS(5);  // 5ms loop for 5ms send interval

    while (1) {
        size_t avail = rx_ring_consume(parse_buf, sizeof(parse_buf));
        if (avail == 0) {
            esp_task_wdt_reset();
            vTaskDelay(parse_delay);
            continue;
        }

        // ADD LOG: Confirm consumption (at INFO level)
        ESP_LOGI("usbhandle", "Consumed %zu bytes from RX ring for parsing", avail);

        size_t parsed = 0;
        while (parsed < avail) {
            uint8_t type;
            uint16_t seq, pay_len, crc;
            frame_parse_status_t status = extract_next_frame(&parse_buf[parsed], avail - parsed, &type, &seq, &pay_len, payload, &crc);
            if (status > 0) {
                parsed += (size_t)status;
                ESP_LOGI("usbhandle", "Parsed frame: type=0x%02X seq=%u len=%u crc=0x%04X", type, seq, pay_len, crc);

                // Handle types: Output DATA to USB (echo CMD if needed)
                if (type == PROTO_TYPE_DATA) {
                    // Unescape if needed (extract_next_frame already unescapes payload)
                    // ESP_LOG_BUFFER_HEX("usbhandle", payload, pay_len);  // Debug: Raw payload
                    
                    // USB output: Queue to CDC ACM
                    size_t written = tinyusb_cdcacm_write_queue(TINYUSB_CDC_ACM_0, payload, pay_len);
                    esp_err_t flush_ret_data = ESP_FAIL;
                    for (int retry = 0; retry < 3; retry++) {
                        flush_ret_data = tinyusb_cdcacm_write_flush(TINYUSB_CDC_ACM_0, pdMS_TO_TICKS(CDC_FLUSH_TIMEOUT_MS));
                        if (flush_ret_data == ESP_OK && written == pay_len) break;
                    }

                    if (flush_ret_data != ESP_OK || written != pay_len) {
                        ESP_LOGE(TAG, "Failed to forward %d bytes from client to USB after retries", pay_len);
                    }
                    
                    // Optional: Send ACK/RESP back
                    // send_reliable(&active_client, PROTO_TYPE_RESP, payload, pay_len);
                } else if (type == PROTO_TYPE_CMD) {
                    // Process command (placeholder)
                    ESP_LOGI("usbhandle", "CMD received, len=%u", pay_len);
                }
            } else if (status == FRAME_INCOMPLETE) {
                // Partial at end: leave in ring next time (but since consumed, actually drop remainder—acceptable for partial)
                ESP_LOGD("usbhandle", "Incomplete frame at offset %zu, dropping %zu bytes", parsed, avail - parsed);
                parsed = avail;  // Advance to drop partial
                break;
            } else {  // FRAME_ERROR (though less common now)
                ESP_LOGW("usbhandle", "Parse error at offset %zu, skipping est %u bytes", parsed, pay_len + 9);
                size_t skip = 7 + pay_len + 2;  // Min header + payload + CRC
                if (skip > avail - parsed) skip = avail - parsed;
                parsed += skip;
                continue;
            }
            esp_task_wdt_reset();
        }
        esp_task_wdt_reset();
        vTaskDelay(parse_delay);
    }
}


