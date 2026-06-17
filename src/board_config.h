// board_config.h - centralna konfiguracja pinow i parametrow stanowiska.
#pragma once

// ---- Pinout (wymagany) ----
#define PIN_STEP 21
#define PIN_DIR 19
#define PIN_EN 18
#define PIN_ENC_A 13
#define PIN_ENC_B 25
#define PIN_LIMIT 22
#define PIN_START 35  // wejscie lokalne, aktywne LOW (GPIO35 = input-only,
                      // wymaga zewnetrznego pull-up)

// ---- Polaryzacje ----
#define LIMIT_ACTIVE_HIGH true   // zadzialanie krancowki => HIGH
#define EN_ACTIVE_HIGH false     // sterownik krokowy: EN zwykle aktywny LOW
#define DIR_INVERT false

// ---- Naped ----
#define MOTOR_MAX_SPEED_HZ 25000.0f
#define MOTOR_MIN_SPEED_HZ 1.0f
#define MOTOR_DEFAULT_ACC 200000u  // [Hz/s]
// Soft-limit pozycji wozka [kroki, +/-]. Blokuje ruch na zewnatrz, zanim
// wozek dojedzie do krancowki. 0 = wylaczony. Wymaga homingu (SET_ZERO).
#define MOTOR_SOFT_LIMIT_STEPS 12000

// ---- UART do PC ----
// UART0 = ten sam port USB co programowanie/konsola. Aby logi ESP_LOG nie
// smiecily w binarnej telemetrii, wyjscie ma dwa tryby (OUT_BINARY/OUT_DEBUG),
// przelaczane komenda SET_OUTPUT_MODE. Baud 460800 daje zapas dla telemetrii
// 250 Hz + komend predkosci z PC.
#define PC_UART_NUM UART_NUM_0
#define PC_UART_TX 1   // domyslny TX0 (mostek USB-UART devkitu)
#define PC_UART_RX 3   // domyslny RX0
#define PC_UART_BAUD 460800

// ---- Timing ----
#define TELEMETRY_PERIOD_US 4000         // 250 Hz (tryb binarny)
#define TELEMETRY_DEBUG_PERIOD_US 100000 // 10 Hz (tryb tekstowy/debug)
#define COMM_TIMEOUT_US 200000           // 200 ms brak komend => bezpieczny stop
#define SAFETY_PERIOD_MS 2               // 500 Hz odczyt LIMIT
#define LIMIT_DEBOUNCE_MS 10             // debounce krancowki (drgania stykow)
#define START_ACTIVE_LOW true            // przelacznik START: ON = zwarcie do GND
#define START_DEBOUNCE_US 20000          // debounce przelacznika START (20 ms)
