// proto - warstwa protokolu ramkowego ESP32 <-> PC
//
// Ramka:
//   [SOF=0xAA][ID][LEN][PAYLOAD ... LEN bajtow][CRC16_LO][CRC16_HI]
//
// CRC16-CCITT (poly 0x1021, init 0xFFFF) liczone po polach [ID, LEN, PAYLOAD],
// przesylane little-endian.
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PROTO_SOF 0xAAu
#define PROTO_MAX_PAYLOAD 64u

// Identyfikatory komunikatow.
typedef enum {
    // Komendy PC -> ESP32 (0x00 - 0x7F)
    PROTO_CMD_PING = 0x01,
    PROTO_CMD_SET_ENABLE = 0x02,    // payload: uint8 (0/1)
    PROTO_CMD_SET_SPEED_HZ = 0x03,  // payload: float32 LE (znak = kierunek)
    PROTO_CMD_SET_ACC = 0x04,       // payload: uint32 LE (Hz/s)
    PROTO_CMD_RESET_FAULT = 0x05,   // payload: brak
    PROTO_CMD_GET_STATUS = 0x06,    // payload: brak
    PROTO_CMD_SET_ZERO = 0x07,      // payload: brak (zeruje pozycje wozka)
    PROTO_CMD_SET_OUTPUT_MODE = 0x08,  // payload: uint8 (0=binary, 1=debug)

    // Odpowiedzi / strumien ESP32 -> PC (0x80 - 0xFF)
    PROTO_MSG_PONG = 0x81,       // payload: brak
    PROTO_MSG_ACK = 0x82,        // payload: uint8 (id potwierdzanej komendy)
    PROTO_MSG_NACK = 0x83,       // payload: uint8 (id odrzuconej komendy)
    PROTO_MSG_TELEMETRY = 0x84,  // payload: proto_telemetry_t
} proto_msg_id_t;

// Payload telemetrii (i odpowiedzi na GET_STATUS). Pakowany dla stalego layoutu.
typedef struct __attribute__((packed)) {
    uint64_t timestamp_us;
    int32_t encoder_count;    // enkoder wahadla (quadrature)
    int32_t position_steps;   // pozycja wozka w krokach (PCNT STEP/DIR)
    float applied_speed_hz;
    uint32_t applied_acc;
    uint8_t limit_state;
    uint8_t fault_state;
    uint8_t drive_enabled;
    uint8_t soft_limit_state;  // 1 = ruch ograniczony soft-limitem pozycji
    uint8_t start_state;       // stan lokalnego przelacznika START (1 = ON)
} proto_telemetry_t;

// Kompletna, zdekodowana ramka.
typedef struct {
    uint8_t id;
    uint8_t len;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_frame_t;

// Stan inkrementalnego parsera (jeden na strumien RX).
typedef struct {
    uint8_t state;
    uint8_t id;
    uint8_t len;
    uint8_t idx;
    uint8_t payload[PROTO_MAX_PAYLOAD];
} proto_parser_t;

// CRC16-CCITT (poly 0x1021, init 0xFFFF).
uint16_t proto_crc16(const uint8_t *data, size_t len);

// Buduje ramke do bufora out (pojemnosc cap).
// Zwraca calkowita dlugosc ramki lub -1 przy bledzie.
int proto_build(uint8_t id, const uint8_t *payload, uint8_t len, uint8_t *out,
                size_t cap);

void proto_parser_init(proto_parser_t *p);

// Podaje pojedynczy bajt do parsera.
// Zwraca true gdy skompletowano ramke z poprawnym CRC (wynik w out).
bool proto_parser_push(proto_parser_t *p, uint8_t byte, proto_frame_t *out);

#ifdef __cplusplus
}
#endif
