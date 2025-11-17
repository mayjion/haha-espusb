#ifndef TCP_CLIENT_H
#define TCP_CLIENT_H
#include <sys/select.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/inet.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_task_wdt.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h> 
#include "gpiohandle.h"
#include "usbhandle.h"  // For tx_ring_t (now pointer-based)
#include "protocol.h"
#include "wificonfig.h"

// Defines (adjusted for client)
#define FRAME_BUFFER_SIZE 2304
#define RX_BUFFER_SIZE FRAME_BUFFER_SIZE
#define SEND_CHECK_INTERVAL_MS 5
#define PROTO_HEARTBEAT_INTERVAL 30000
#define PORT 12345
#define SERVER_IP "192.168.4.1"  // Default AP IP for server
#define RX_RING_SIZE 16*1024  // Ring buffer for incoming TCP data (larger than TX for bursts)

// Ring buffer struct for RX (mirrors tx_ring_t from usbhandle; buf now pointer)
typedef struct {
    uint8_t *buf;  // Pointer for dynamic alloc
    size_t head;  // Write position
    size_t tail;  // Read position
    size_t len;
    size_t size;  // Track allocated size (for safety)
    SemaphoreHandle_t mutex;  // Protect access
} rx_ring_t;

typedef struct {
    int sock;
    bool active;
    struct sockaddr_in addr;
    uint16_t seq_tx;
    uint16_t seq_rx;
    uint32_t last_heartbeat;
    bool pending;
    uint16_t pending_seq;
    uint8_t pending_payload[MAX_TX_SIZE];
    uint16_t pending_len;
    uint8_t pending_type;
    TickType_t last_send_time;
    int pending_retries;
} client_t;

extern client_t active_client;
extern SemaphoreHandle_t client_mutex;
extern rx_ring_t rx_ring;  // Shared RX ring for TCP -> parse task
extern SemaphoreHandle_t rx_mutex;  // For rx_ring protection
// tx_ring extern via usbhandle.h

void app_main(void);
void tcp_client_task(void *pvParameters);
void send_task(void *pvParameters);
void parse_and_usb_task(void *pvParameters);  // Parse RX ring and output to USB
esp_err_t send_reliable(client_t *client, uint8_t type, const uint8_t *payload, uint16_t payload_len);
esp_err_t rx_ring_append(const uint8_t *data, size_t len);  // Append to RX ring
size_t rx_ring_consume(uint8_t *buf, size_t max_len);  // Consume from RX ring
esp_err_t send_control_to_tcp(uint8_t type, const uint8_t *payload, uint16_t payload_len);

#endif