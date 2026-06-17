#include "proto.h"

#include <string.h>

// Stany parsera ramek.
enum {
    ST_SOF = 0,
    ST_ID,
    ST_LEN,
    ST_PAYLOAD,
    ST_CRC_LO,
    ST_CRC_HI,
};

uint16_t proto_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

int proto_build(uint8_t id, const uint8_t *payload, uint8_t len, uint8_t *out,
                size_t cap)
{
    if (out == NULL) {
        return -1;
    }
    if (len > PROTO_MAX_PAYLOAD) {
        return -1;
    }
    if (len > 0 && payload == NULL) {
        return -1;
    }
    const size_t total = (size_t)len + 5u;  // SOF+ID+LEN+payload+CRC16
    if (cap < total) {
        return -1;
    }

    out[0] = PROTO_SOF;
    out[1] = id;
    out[2] = len;
    if (len > 0) {
        memcpy(&out[3], payload, len);
    }

    // CRC po [ID, LEN, PAYLOAD].
    const uint16_t crc = proto_crc16(&out[1], (size_t)len + 2u);
    out[3 + len] = (uint8_t)(crc & 0xFFu);
    out[4 + len] = (uint8_t)((crc >> 8) & 0xFFu);

    return (int)total;
}

void proto_parser_init(proto_parser_t *p)
{
    if (p == NULL) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->state = ST_SOF;
}

bool proto_parser_push(proto_parser_t *p, uint8_t byte, proto_frame_t *out)
{
    if (p == NULL || out == NULL) {
        return false;
    }

    static uint16_t rx_crc;  // tylko skladanie 2 bajtow CRC miedzy wywolaniami

    switch (p->state) {
        case ST_SOF:
            if (byte == PROTO_SOF) {
                p->state = ST_ID;
            }
            break;

        case ST_ID:
            p->id = byte;
            p->state = ST_LEN;
            break;

        case ST_LEN:
            p->len = byte;
            p->idx = 0;
            if (p->len > PROTO_MAX_PAYLOAD) {
                // Bledna dlugosc -> odrzuc i wroc do synchronizacji.
                p->state = ST_SOF;
            } else if (p->len == 0) {
                p->state = ST_CRC_LO;
            } else {
                p->state = ST_PAYLOAD;
            }
            break;

        case ST_PAYLOAD:
            p->payload[p->idx++] = byte;
            if (p->idx >= p->len) {
                p->state = ST_CRC_LO;
            }
            break;

        case ST_CRC_LO:
            rx_crc = byte;
            p->state = ST_CRC_HI;
            break;

        case ST_CRC_HI: {
            rx_crc |= (uint16_t)byte << 8;
            p->state = ST_SOF;

            // Przelicz CRC po [ID, LEN, PAYLOAD] na ciaglym buforze.
            uint8_t tmp[PROTO_MAX_PAYLOAD + 2];
            tmp[0] = p->id;
            tmp[1] = p->len;
            if (p->len > 0) {
                memcpy(&tmp[2], p->payload, p->len);
            }
            uint16_t calc = proto_crc16(tmp, (size_t)p->len + 2u);

            if (calc == rx_crc) {
                out->id = p->id;
                out->len = p->len;
                if (p->len > 0) {
                    memcpy(out->payload, p->payload, p->len);
                }
                return true;
            }
            break;
        }

        default:
            p->state = ST_SOF;
            break;
    }

    return false;
}
