#ifndef USBHANDLE_H
#define USBHANDLE_H

#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tcp_client.h"  // For shared queue and TAG

#define CONFIG_TINYUSB_CDC_RX_BUFSIZE 512
#define CDC_FLUSH_TIMEOUT_MS 50
#define DEFAULT_BAUD_RATE 115200  // Updated to target baud rate
#define QUEUE_SIZE 300
#define UART_BUF_SIZE 512
#define ACK_TIMEOUT_MS 200
#define MAX_TX_SIZE 1024
#define TX_BUFFER_SIZE 10*1024

typedef struct {
    uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE];
    size_t buf_len;
    int itf;
} app_message_t;

// Ring buffer for TX data (buf now pointer for heap alloc)
typedef struct {
    uint8_t *buf;  // CHANGED: Pointer for dynamic alloc
    size_t head;  // Write position
    size_t tail;  // Read position
    size_t len;   // Current length
    size_t size;  // CHANGED: Track allocated size (for safety)
} tx_ring_t;

extern QueueHandle_t app_queue;
extern tx_ring_t tx_ring;
esp_err_t init_usb(void);
void usb_to_tcp_task(void *pvParameters);  // Added: Prototype

#endif
