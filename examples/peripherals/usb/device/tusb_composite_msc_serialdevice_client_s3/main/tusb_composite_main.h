// tusb_composite_client.h (Client Header)
#ifndef _TUSB_COMPOSITE_CLIENT_H_
#define _TUSB_COMPOSITE_CLIENT_H_

#include <stdio.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"  // For WDT functions

// Define if not present
#define RX_BUFFER_SIZE 2048

// Protocol Constants (same as server)
#define PROTO_HEADER 0xFFFF
#define PROTO_TYPE_DATA 0x00
#define PROTO_TYPE_CMD 0x01
#define PROTO_TYPE_RESP 0x02
#define PROTO_TYPE_HEARTBEAT 0x03
#define PROTO_SEQ_MAX 0xFFFF
#define PROTO_CRC_POLY 0x1021
#define PROTO_CRC_INIT 0xFFFF
#define PROTO_ESCAPE 0xFE
#define PROTO_ESC_FF 0xDF
#define PROTO_ESC_FE 0xDE
#define PROTO_TIMEOUT_MS 1000
#define PROTO_MAX_RETRIES 3
#define PROTO_HEARTBEAT_INTERVAL 30000

#define QUEUE_SIZE 50
#define CONFIG_TINYUSB_CDC_RX_BUFSIZE 512
#define SERVER_SSID "FUNLIGHT"
#define SERVER_PASS "funlight"
#define SERVER_IP "192.168.4.1"
#define PORT 12345
#define UART_BUF_SIZE 512
#define CDC_FLUSH_TIMEOUT_MS 50
#define DEFAULT_BAUD_RATE 115200
#define LED_GPIO GPIO_NUM_7
#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_LONG_PRESS_MS 5000

typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t buf_len;
    int itf;
} app_message_t;

typedef struct {
    int sock;              // Server socket
    bool active;           // Connection active
    uint16_t seq_tx;       // Send sequence
    uint16_t seq_rx;       // Expected receive sequence
    uint32_t last_heartbeat; // Last heartbeat time
    uint16_t last_tx_seq;  // 新增：最后发送 seq
    uint8_t last_payload[512];  // 新增：最后 payload
    uint16_t last_plen;    // 新增：最后 payload len
} client_t;

// Protocol functions
uint16_t compute_crc16(const uint8_t *data, size_t len);
void escape_bytes(uint8_t *data, size_t *len);
bool unescape_bytes(uint8_t *data, size_t *len);
bool parse_frame(const uint8_t *buf, size_t len, uint8_t *type, uint16_t *seq, uint16_t *payload_len, uint8_t *payload, uint16_t *crc);
size_t build_frame(uint8_t *buf, uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t payload_len, uint16_t *out_crc);
esp_err_t send_reliable(client_t *client, uint8_t type, const uint8_t *payload, uint16_t payload_len);
bool handle_response(client_t *client, uint8_t *payload, uint16_t payload_len);
int extract_next_frame(uint8_t *buf, size_t buf_len, uint8_t *type, uint16_t *seq, uint16_t *payload_len, uint8_t *payload, uint16_t *crc);

void wifi_init_sta(void);
esp_err_t tcp_client_connect(void);

#endif