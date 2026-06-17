// app_main + io_task (warstwa komunikacji UART <-> PC).
//
// Architektura (FreeRTOS):
//   - io_task     : parsuje ramki RX, waliduje CRC, publikuje komendy do kolejki,
//                   wysyla telemetrie TX, pilnuje timeoutu komunikacji.
//   - motor_task  : konsumuje komendy, prowadzi rampe, egzekwuje blokade fault.
//   - safety_task : cykliczny odczyt LIMIT + zatrzask fault.
//   - encoder     : ISR GPIO + atomowy snapshot licznika.
//
// ESP32 NIE liczy regulatora wahadla - to tylko warstwa czasu rzeczywistego I/O.
#include <stdio.h>
#include <string.h>

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "motor.h"
#include "proto.h"
#include "safety.h"

static const char *TAG = "app";

// === PROSTY TEST STEP ===
// Ustaw na 1, aby uruchomic TYLKO goly LEDC na pinie STEP (bez motor/PCNT/
// enable/taskow). Sluzy do izolacji: czy na GPIO21 w ogole pojawia sie zegar.
// Po tescie wroc do 0 i wgraj ponownie.
#define STEP_LEDC_SELFTEST 0

// Tryb wyjscia na wspolnej magistrali UART0/USB:
//  OUT_BINARY - czyste ramki binarne (aplikacja PC), logi ESP_LOG wyciszone,
//  OUT_DEBUG  - czytelny tekst + logi ESP_LOG (np. pio device monitor).
typedef enum {
    OUT_BINARY = 0,
    OUT_DEBUG = 1,
} output_mode_t;

// Domyslnie DEBUG: po starcie zwykly monitor pokazuje czytelne dane i logi.
// Aplikacja PC sama przelacza na OUT_BINARY (pc_console robi to automatycznie).
static volatile output_mode_t s_out_mode = OUT_DEBUG;

// Stan lokalnego przelacznika START (aktualizowany w io_task, czytany w telemetrii).
static volatile bool s_start_on = false;

static QueueHandle_t s_cmd_queue;

// Odczyt surowego stanu przelacznika START (ON wg polaryzacji).
static inline bool start_raw_on(void)
{
    const int lvl = gpio_get_level((gpio_num_t)PIN_START);
    return START_ACTIVE_LOW ? (lvl == 0) : (lvl != 0);
}

static void apply_output_mode(output_mode_t mode)
{
    s_out_mode = mode;
    // W trybie binarnym wyciszamy logi, by nie psuc strumienia ramek.
    esp_log_level_set("*", mode == OUT_BINARY ? ESP_LOG_NONE : ESP_LOG_INFO);
}

// ---- TX helpers ----
static void send_frame(uint8_t id, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[PROTO_MAX_PAYLOAD + 5];
    const int n = proto_build(id, payload, len, buf, sizeof(buf));
    if (n > 0) {
        uart_write_bytes(PC_UART_NUM, (const char *)buf, (size_t)n);
    }
}

static void fill_telemetry(proto_telemetry_t *t)
{
    t->timestamp_us = (uint64_t)esp_timer_get_time();
    t->encoder_count = encoder_get_count();
    t->position_steps = motor_get_position_steps();
    t->applied_speed_hz = motor_get_applied_speed_hz();
    t->applied_acc = motor_get_applied_acc();
    t->limit_state = safety_get_limit_state() ? 1 : 0;
    t->fault_state = safety_is_fault() ? 1 : 0;
    t->drive_enabled = motor_is_enabled() ? 1 : 0;
    t->soft_limit_state = motor_soft_limit_active() ? 1 : 0;
    t->start_state = s_start_on ? 1 : 0;
}

static void send_telemetry_binary(uint8_t id)
{
    proto_telemetry_t t;
    fill_telemetry(&t);
    send_frame(id, (const uint8_t *)&t, (uint8_t)sizeof(t));
}

static void send_telemetry_text(void)
{
    proto_telemetry_t t;
    fill_telemetry(&t);
    char line[160];
    const int n = snprintf(
        line, sizeof(line),
        "TLM ts=%llu enc=%ld pos=%ld spd=%.1fHz acc=%lu lim=%u flt=%u en=%u "
        "soft=%u start=%u\r\n",
        (unsigned long long)t.timestamp_us, (long)t.encoder_count,
        (long)t.position_steps, t.applied_speed_hz,
        (unsigned long)t.applied_acc, t.limit_state, t.fault_state,
        t.drive_enabled, t.soft_limit_state, t.start_state);
    if (n > 0) {
        uart_write_bytes(PC_UART_NUM, line, (size_t)n);
    }
}

// ---- Obsluga pojedynczej ramki komendy ----
static void handle_frame(const proto_frame_t *f)
{
    motor_cmd_t cmd;
    bool push = false;

    switch (f->id) {
        case PROTO_CMD_PING:
            send_frame(PROTO_MSG_PONG, NULL, 0);
            return;

        case PROTO_CMD_GET_STATUS:
            if (s_out_mode == OUT_BINARY) {
                send_telemetry_binary(PROTO_MSG_TELEMETRY);
            } else {
                send_telemetry_text();
            }
            return;

        case PROTO_CMD_SET_OUTPUT_MODE:
            if (f->len != 1) {
                goto nack;
            }
            apply_output_mode(f->payload[0] ? OUT_DEBUG : OUT_BINARY);
            send_frame(PROTO_MSG_ACK, &f->id, 1);
            return;

        case PROTO_CMD_SET_ENABLE:
            if (f->len != 1) {
                goto nack;
            }
            cmd.type = MOTOR_CMD_SET_ENABLE;
            cmd.arg.b = (f->payload[0] != 0);
            push = true;
            break;

        case PROTO_CMD_SET_SPEED_HZ:
            if (f->len != 4) {
                goto nack;
            }
            cmd.type = MOTOR_CMD_SET_SPEED;
            memcpy(&cmd.arg.f, f->payload, 4);
            push = true;
            break;

        case PROTO_CMD_SET_ACC:
            if (f->len != 4) {
                goto nack;
            }
            cmd.type = MOTOR_CMD_SET_ACC;
            memcpy(&cmd.arg.u, f->payload, 4);
            push = true;
            break;

        case PROTO_CMD_RESET_FAULT:
            cmd.type = MOTOR_CMD_RESET_FAULT;
            push = true;
            break;

        case PROTO_CMD_SET_ZERO:
            cmd.type = MOTOR_CMD_ZERO_POSITION;
            push = true;
            break;

        default:
            goto nack;
    }

    if (push) {
        xQueueSend(s_cmd_queue, &cmd, 0);
        const uint8_t acked = f->id;
        send_frame(PROTO_MSG_ACK, &acked, 1);
    }
    return;

nack : {
    const uint8_t nid = f->id;
    send_frame(PROTO_MSG_NACK, &nid, 1);
}
}

static void io_task(void *arg)
{
    (void)arg;
    proto_parser_t parser;
    proto_parser_init(&parser);

    uint8_t rxbuf[256];
    int64_t last_tlm = esp_timer_get_time();
    int64_t last_cmd = esp_timer_get_time();
    bool timed_out = false;

    // Przelacznik START: debounce czasowy + detekcja zbocza. Stan poczatkowy
    // przyjmujemy bez wysylania enable (brak auto-enable po starcie).
    bool start_confirmed = start_raw_on();
    bool start_raw_prev = start_confirmed;
    int64_t start_change_us = esp_timer_get_time();
    s_start_on = start_confirmed;

    for (;;) {
        const int n = uart_read_bytes(PC_UART_NUM, rxbuf, sizeof(rxbuf),
                                      pdMS_TO_TICKS(5));
        for (int i = 0; i < n; ++i) {
            proto_frame_t f;
            if (proto_parser_push(&parser, rxbuf[i], &f)) {
                last_cmd = esp_timer_get_time();
                timed_out = false;
                handle_frame(&f);
            }
        }

        const int64_t now = esp_timer_get_time();

        // Przelacznik START (bistabilny): na potwierdzonej zmianie stanu
        // ustaw enable napedu (przez kolejke, jak SET_ENABLE z PC).
        const bool start_raw = start_raw_on();
        if (start_raw != start_raw_prev) {
            start_raw_prev = start_raw;
            start_change_us = now;
        }
        if (start_raw == start_confirmed) {
            // zgodny ze stanem ustalonym - nic
        } else if ((now - start_change_us) >= START_DEBOUNCE_US) {
            start_confirmed = start_raw;
            s_start_on = start_confirmed;
            motor_cmd_t ec = {.type = MOTOR_CMD_SET_ENABLE};
            ec.arg.b = start_confirmed;
            xQueueSend(s_cmd_queue, &ec, 0);
        }

        // Timeout komunikacji -> bezpieczny stop (jednokrotnie).
        if (!timed_out && (now - last_cmd) > COMM_TIMEOUT_US) {
            timed_out = true;
            motor_cmd_t c = {.type = MOTOR_CMD_STOP};
            xQueueSend(s_cmd_queue, &c, 0);
        }

        // Telemetria okresowa: binarna ~100 Hz, tekstowa ~10 Hz.
        const int64_t period = (s_out_mode == OUT_BINARY)
                                   ? TELEMETRY_PERIOD_US
                                   : TELEMETRY_DEBUG_PERIOD_US;
        if (now - last_tlm >= period) {
            last_tlm = now;
            if (s_out_mode == OUT_BINARY) {
                send_telemetry_binary(PROTO_MSG_TELEMETRY);
            } else {
                send_telemetry_text();
            }
        }
    }
}

static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = PC_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    ESP_ERROR_CHECK(uart_driver_install(PC_UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PC_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PC_UART_NUM, PC_UART_TX, PC_UART_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

#if STEP_LEDC_SELFTEST
// Goly test: staly prostokat 1 kHz, 50% na pinie STEP. Nic wiecej.
static void step_ledc_selftest(void)
{
    ESP_LOGI(TAG, "STEP LEDC selftest: 1 kHz na GPIO%d", PIN_STEP);

    ledc_timer_config_t t = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));

    ledc_channel_config_t c = {
        .gpio_num = PIN_STEP,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 512,  // 512/1024 = 50%
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Inverted pendulum I/O layer - start");

#if STEP_LEDC_SELFTEST
    step_ledc_selftest();  // nie wraca
    return;
#endif

    // Wejscie lokalne START (GPIO35 = input-only, brak pull wewnetrznego).
    gpio_config_t start_io = {
        .pin_bit_mask = (1ULL << PIN_START),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&start_io);

    // Serwis ISR GPIO (wspoldzielony) - musi byc przed encoder_init.
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    encoder_init(PIN_ENC_A, PIN_ENC_B);

    const motor_config_t mcfg = {
        .step_pin = PIN_STEP,
        .dir_pin = PIN_DIR,
        .en_pin = PIN_EN,
        .en_active_high = EN_ACTIVE_HIGH,
        .dir_invert = DIR_INVERT,
        .max_speed_hz = MOTOR_MAX_SPEED_HZ,
        .min_speed_hz = MOTOR_MIN_SPEED_HZ,
        .default_acc = MOTOR_DEFAULT_ACC,
        .soft_limit_steps = MOTOR_SOFT_LIMIT_STEPS,
    };
    motor_init(&mcfg);

    const safety_config_t scfg = {
        .limit_pin = PIN_LIMIT,
        .limit_active_high = LIMIT_ACTIVE_HIGH,
        .period_ms = SAFETY_PERIOD_MS,
        .debounce_ms = LIMIT_DEBOUNCE_MS,
    };
    safety_init(&scfg);

    uart_init();
    apply_output_mode(s_out_mode);  // start w trybie DEBUG (tekst + logi)
    ESP_LOGI(TAG, "Output mode = DEBUG. PC app przelacza na BINARY (cmd 0x08).");

    s_cmd_queue = xQueueCreate(16, sizeof(motor_cmd_t));
    configASSERT(s_cmd_queue != NULL);

    // safety (najwyzszy priorytet) i motor na rdzeniu 1; io na rdzeniu 0.
    xTaskCreatePinnedToCore(safety_task, "safety", 3072, NULL, 6, NULL, 1);
    xTaskCreatePinnedToCore(motor_task, "motor", 4096, s_cmd_queue, 5, NULL, 1);
    xTaskCreatePinnedToCore(io_task, "io", 4096, NULL, 4, NULL, 0);
}
