#include "protocol.h"
#include <string.h>  // Added for memcpy

static uint8_t frame_buffer[FRAME_BUFFER_SIZE];
static uint8_t escape_temp[FRAME_BUFFER_SIZE];

uint16_t compute_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = PROTO_CRC_INIT;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ PROTO_CRC_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void escape_bytes(uint8_t *data, uint16_t *len) {
    uint8_t temp[MAX_TX_SIZE*2 + 2];  // Temp buffer for safe escaping
    size_t orig_len = *len;
    size_t j = 0;
    for (size_t i = 0; i < orig_len; ++i) {
        if (data[i] == 0xFF) {
            temp[j++] = PROTO_ESCAPE;
            temp[j++] = PROTO_ESC_FF;
        } else if (data[i] == PROTO_ESCAPE) {
            temp[j++] = PROTO_ESCAPE;
            temp[j++] = PROTO_ESC_FE;
        } else {
            temp[j++] = data[i];
        }
    }
    memcpy(data, temp, j);
    *len = j;
}

size_t build_escaped_frame(uint8_t *frame, uint8_t type, uint16_t seq, const uint8_t *payload, uint16_t payload_len, uint16_t *crc_out) {
    // 清零 frame 缓冲以避免垃圾内存
    memset(frame, 0, FRAME_BUFFER_SIZE);
    // 默认序列号
    seq = 0;
    // Build header (unescaped len)
    frame[0] = 0xFF;
    frame[1] = 0xFF;
    frame[2] = type;
    frame[3] = (seq >> 8) & 0xFF;
    frame[4] = seq & 0xFF;
    frame[5] = (payload_len >> 8) & 0xFF;
    frame[6] = payload_len & 0xFF;

    // Compute CRC over unescaped: header fields + original payload
    uint8_t crc_data[5 + MAX_TX_SIZE];
    crc_data[0] = type;
    crc_data[1] = (seq >> 8) & 0xFF;
    crc_data[2] = seq & 0xFF;
    crc_data[3] = (payload_len >> 8) & 0xFF;
    crc_data[4] = payload_len & 0xFF;
    memcpy(crc_data + 5, payload, payload_len);  // Original unescaped
    uint16_t crc = compute_crc16(crc_data, 5 + payload_len);
    if (crc_out) *crc_out = crc;

    // Escape payload for TX
    uint8_t escaped_payload[MAX_TX_SIZE * 2];
    memcpy(escaped_payload, payload, payload_len);
    uint16_t esc_len = payload_len;
    escape_bytes(escaped_payload, &esc_len);

    // Append escaped payload
    memcpy(&frame[7], escaped_payload, esc_len);

    // Escape and append CRC (修复：使用独立缓冲避免溢出)
    uint8_t temp_crc[2] = {(crc >> 8) & 0xFF, crc & 0xFF};
    uint8_t escaped_crc[4];  // 足够大缓冲 (max 4 字节 escaped)
    memcpy(escaped_crc, temp_crc, 2);
    uint16_t crc_esc_len = 2;
    escape_bytes(escaped_crc, &crc_esc_len);
    memcpy(&frame[7 + esc_len], escaped_crc, crc_esc_len);

    return 7 + esc_len + crc_esc_len;
}

bool unescape_bytes(uint8_t *data, size_t *len) {
    uint8_t temp[FRAME_BUFFER_SIZE + 32];  // Use FRAME_BUFFER_SIZE for consistency
    size_t orig_len = *len;
    size_t j = 0;
    size_t i = 0;
    while (i < orig_len) {
        if (data[i] == PROTO_ESCAPE) {
            if (i + 1 < orig_len) {
                ++i;  // Advance to escape code byte
                if (data[i] == PROTO_ESC_FF) {
                    temp[j++] = 0xFF;
                } else if (data[i] == PROTO_ESC_FE) {
                    temp[j++] = PROTO_ESCAPE;
                } else {
                    // Invalid escape, copy as-is
                    temp[j++] = PROTO_ESCAPE;
                }
            } else {
                temp[j++] = data[i];
            }
            ++i;
        } else {
            temp[j++] = data[i++];
        }
    }
    memcpy(data, temp, j);
    *len = j;
    return true;
}


// Optimized extract_next_frame: Better skip estimation, explicit status returns
frame_parse_status_t extract_next_frame(uint8_t *buf, size_t buf_len, uint8_t *type, uint16_t *seq, uint16_t *payload_len, uint8_t *payload, uint16_t *crc) {
    if (buf_len < 7) return FRAME_INCOMPLETE;

    size_t i = 0;
    while (i + 6 < buf_len) {
        if (buf[i] == 0xFF && buf[i + 1] == 0xFF) {
            *type = buf[i + 2];
            *seq = (buf[i + 3] << 8) | buf[i + 4];
            *payload_len = (buf[i + 5] << 8) | buf[i + 6];

            size_t header_end = i + 7;
            if (header_end > buf_len) {
                return FRAME_INCOMPLETE;  // Wait for more
            }

            // Unescape payload (optimized: tighter bounds check)
            size_t unesc_count = 0;
            size_t raw_pos = header_end;
            size_t j = 0;
            bool parse_fail = false;
            while (raw_pos < buf_len && unesc_count < *payload_len) {
                if (buf[raw_pos] == PROTO_ESCAPE) {
                    if (raw_pos + 1 >= buf_len) {
                        parse_fail = true;
                        break;
                    }
                    uint8_t esc_code = buf[raw_pos + 1];
                    raw_pos += 2;
                    if (esc_code == PROTO_ESC_FF) {
                        payload[j++] = 0xFF;
                    } else if (esc_code == PROTO_ESC_FE) {
                        payload[j++] = PROTO_ESCAPE;
                    } else {
                        parse_fail = true;
                        break;
                    }
                    unesc_count++;
                } else {
                    payload[j++] = buf[raw_pos++];
                    unesc_count++;
                }
            }
            if (parse_fail || unesc_count < *payload_len) {
                // Improved skip: Actual consumed + estimated remaining (payload*1.5 for escapes + CRC*2)
                size_t est_remaining = (*payload_len - unesc_count) * 1.5 + 4;
                i += (header_end - i) + est_remaining;
                if (i > buf_len) i = buf_len;
                continue;
            }

            // Unescape CRC (similar optimization)
            uint8_t crc_unesc[2];
            unesc_count = 0;
            size_t k = 0;
            parse_fail = false;
            while (raw_pos < buf_len && unesc_count < 2) {
                if (buf[raw_pos] == PROTO_ESCAPE) {
                    if (raw_pos + 1 >= buf_len) {
                        parse_fail = true;
                        break;
                    }
                    uint8_t esc_code = buf[raw_pos + 1];
                    raw_pos += 2;
                    if (esc_code == PROTO_ESC_FF) {
                        crc_unesc[k++] = 0xFF;
                    } else if (esc_code == PROTO_ESC_FE) {
                        crc_unesc[k++] = PROTO_ESCAPE;
                    } else {
                        parse_fail = true;
                        break;
                    }
                    unesc_count++;
                } else {
                    crc_unesc[k++] = buf[raw_pos++];
                    unesc_count++;
                }
            }
            if (parse_fail || unesc_count < 2) {
                size_t est_remaining = (2 - unesc_count) * 2 + (*payload_len % 2);  // CRC margin
                i += (raw_pos - i) + est_remaining;
                if (i > buf_len) i = buf_len;
                continue;
            }

            *crc = (crc_unesc[0] << 8) | crc_unesc[1];
            // Verify CRC using unescaped data
            uint8_t temp_payload[FRAME_BUFFER_SIZE];
            memcpy(temp_payload, payload, *payload_len);
            uint8_t calc_data[5 + FRAME_BUFFER_SIZE];
            calc_data[0] = *type;
            calc_data[1] = (*seq >> 8) & 0xFF;
            calc_data[2] = *seq & 0xFF;
            calc_data[3] = (*payload_len >> 8) & 0xFF;
            calc_data[4] = *payload_len & 0xFF;
            memcpy(calc_data + 5, temp_payload, *payload_len);
            uint16_t calc_crc = compute_crc16(calc_data, 5 + *payload_len);
            if (calc_crc != *crc) {
                ESP_LOGI("PROTO", "CRC mismatch: calc=0x%04X, recv=0x%04X", calc_crc, *crc);
                // ADDED: Skip this frame like unescape fail
                size_t frame_est = 7 + (*payload_len) * 1.5 + 4;  // Header + esc payload est + esc CRC
                i += frame_est;
                if (i > buf_len) i = buf_len;
                continue;  // Scan for next header
            }
            return (frame_parse_status_t)(raw_pos - i);  // Bytes consumed (positive = success)
        }
        i++;
    }
    return FRAME_INCOMPLETE;  // No full frame
}

bool parse_frame(const uint8_t *buf, size_t len, uint8_t *type, uint16_t *seq, uint16_t *payload_len, uint8_t *payload, uint16_t *crc) {
    if (len < 7) return false;  // Min frame size
    if (buf[0] != 0xFF || buf[1] != 0xFF) return false;
    *type = buf[2];
    *seq = (buf[3] << 8) | buf[4];
    *payload_len = (buf[5] << 8) | buf[6];
    if (len < 7 + *payload_len + 2) return false;
    memcpy(payload, &buf[7], *payload_len);
    *crc = (buf[7 + *payload_len] << 8) | buf[7 + *payload_len + 1];
    uint16_t calc_crc = compute_crc16(&buf[2], 1 + 2 + 2 + *payload_len);  // Type + Seq + Len + Payload
    return calc_crc == *crc;
}