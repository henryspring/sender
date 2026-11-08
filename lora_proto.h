#ifndef LORA_PROTO_H
#define LORA_PROTO_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Keep this format identical to the working ESP32 receiver. */
#define LORA_SYNC      0xAAu
#define LORA_F_SYN     0x01u
#define LORA_F_ACK     0x02u
#define LORA_F_END     0x04u
#define LORA_F_REQ     0x08u
#define LORA_F_RST     0x10u
#define LORA_F_HELLO   0x20u

#define LORA_PAYLOAD   48u
#define LORA_HDR       6u
#define LORA_FRAME_MAX (LORA_HDR + LORA_PAYLOAD + 2u)
#define LORA_WINDOW    32u

typedef struct {
    uint8_t flags;
    uint8_t id;
    uint16_t seq;
    uint8_t len;
    uint8_t payload[LORA_PAYLOAD];
} LoraFrame;

typedef struct {
    uint8_t buf[LORA_FRAME_MAX];
    uint8_t n;
} LoraParser;

static inline uint16_t lora_crc16(const uint8_t *data, size_t n)
{
    uint16_t crc = 0xFFFFu;
    while (n--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static inline uint32_t lora_crc32(const uint8_t *data, size_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    while (n--) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1u) {
                crc = (crc >> 1) ^ 0xEDB88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

static inline void lora_put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24);
    p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);
    p[3] = (uint8_t)value;
}

static inline uint32_t lora_get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static inline size_t lora_build_frame(uint8_t *out, uint8_t flags, uint8_t image_id,
                                      uint16_t seq, const uint8_t *data, uint8_t len)
{
    out[0] = LORA_SYNC;
    out[1] = flags;
    out[2] = image_id;
    out[3] = (uint8_t)(seq >> 8);
    out[4] = (uint8_t)seq;
    out[5] = len;

    if (len != 0u) {
        memcpy(out + LORA_HDR, data, len);
    }

    uint16_t crc = lora_crc16(out + 1, 5u + len);
    out[LORA_HDR + len] = (uint8_t)(crc >> 8);
    out[LORA_HDR + len + 1u] = (uint8_t)crc;
    return LORA_HDR + len + 2u;
}

static inline int lora_parser_feed(LoraParser *parser, uint8_t byte, LoraFrame *out)
{
    if (parser->n == 0u && byte != LORA_SYNC) {
        return 0;
    }

    parser->buf[parser->n++] = byte;
    if (parser->n < LORA_HDR) {
        return 0;
    }

    uint8_t length = parser->buf[5];
    if (length > LORA_PAYLOAD) {
        parser->n = 0u;
        return 0;
    }

    uint8_t total = (uint8_t)(LORA_HDR + length + 2u);
    if (parser->n < total) {
        return 0;
    }
    parser->n = 0u;

    uint16_t received_crc = ((uint16_t)parser->buf[total - 2u] << 8) |
                            parser->buf[total - 1u];
    if (lora_crc16(parser->buf + 1, total - 3u) != received_crc) {
        return 0;
    }

    out->flags = parser->buf[1];
    out->id = parser->buf[2];
    out->seq = ((uint16_t)parser->buf[3] << 8) | parser->buf[4];
    out->len = length;
    memcpy(out->payload, parser->buf + LORA_HDR, length);
    return 1;
}

#endif
