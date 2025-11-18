#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_log.h"

// Protocol Constants (unchanged)
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
#define MAX_TX_SIZE 1024
#define FRAME_BUFFER_SIZE 2304

// 在 protocol.h 末尾添加（extern for sharing）
extern uint8_t g_frame_buffer[FRAME_BUFFER_SIZE];
extern uint8_t g_escaped_payload[MAX_TX_SIZE * 2];
extern uint8_t g_crc_data[5 + MAX_TX_SIZE];

// Function prototypes (added extract_next_frame return codes)
typedef enum {
    FRAME_OK = 1,      // Full frame parsed
    FRAME_INCOMPLETE = 0, // Need more data
    FRAME_ERROR = -1   // Bad frame, skipped
} frame_parse_status_t;

uint16_t compute_crc16(const uint8_t *data, size_t len);
void escape_bytes(uint8_t *data, uint16_t *len);
frame_parse_status_t extract_next_frame(uint8_t *buf, size_t buf_len, uint8_t *type, uint16_t *seq, uint16_t *payload_len, uint8_t *payload, uint16_t *crc);  // Updated return type
bool unescape_bytes(uint8_t *data, size_t *len);
bool parse_frame(const uint8_t *buf, size_t len, uint8_t *type, uint16_t *seq, uint16_t *payload_len, uint8_t *payload, uint16_t *crc);
size_t build_escaped_frame(uint8_t *frame, uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t payload_len, uint16_t *crc_out);

#endif