#include "motor.h"

#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "safety.h"
#include "soc/gpio_sig_map.h"

// Konfiguracja LEDC dla generacji STEP.
#define MOTOR_LEDC_MODE LEDC_HIGH_SPEED_MODE
#define MOTOR_LEDC_TIMER LEDC_TIMER_0
#define MOTOR_LEDC_CHANNEL LEDC_CHANNEL_0
// 11-bit @ APB 80 MHz: zakres ~38 Hz .. ~39 kHz (cap 25 kHz miesci sie z
// zapasem). Wyzsza rozdzielczosc obnizylaby maks. czestotliwosc ponizej 25 kHz.
#define MOTOR_LEDC_RES LEDC_TIMER_11_BIT
#define MOTOR_LEDC_DUTY_50 (1u << (MOTOR_LEDC_RES - 1))  // 50% wypelnienia

// PCNT - sprzetowy licznik impulsow STEP (kierunek z DIR).
// Licznik HW jest 16-bit, wiec na progach +/-H_LIM akumulujemy do 32-bit.
#define MOTOR_PCNT_UNIT PCNT_UNIT_0
#define MOTOR_PCNT_H_LIM 10000

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_pos_mux = portMUX_INITIALIZER_UNLOCKED;

static motor_config_t s_cfg;
static float s_target_hz = 0.0f;    // zadana (znak = kierunek)
static float s_applied_hz = 0.0f;   // aktualnie na sprzecie (znak = kierunek)
static uint32_t s_acc = 0;          // [Hz/s]
static bool s_enabled = false;
static bool s_pulses_on = false;
static volatile int32_t s_pos_accum = 0;   // akumulator przepelnien PCNT
static volatile int32_t s_pos_offset = 0;  // przesuniecie zera (homing)
static volatile bool s_soft_limit_hit = false;

static inline void en_pin_write(bool active)
{
    const int lvl = s_cfg.en_active_high ? (active ? 1 : 0) : (active ? 0 : 1);
    gpio_set_level((gpio_num_t)s_cfg.en_pin, lvl);
}

static inline void dir_pin_write(bool forward)
{
    int lvl = forward ? 1 : 0;
    if (s_cfg.dir_invert) {
        lvl = !lvl;
    }
    gpio_set_level((gpio_num_t)s_cfg.dir_pin, lvl);
}

static void pulses_stop(void)
{
    if (s_pulses_on) {
        ledc_stop(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL, 0);
        s_pulses_on = false;
    }
}

static void pulses_set(float freq_hz)
{
    if (freq_hz < s_cfg.min_speed_hz) {
        pulses_stop();
        return;
    }
    ledc_set_freq(MOTOR_LEDC_MODE, MOTOR_LEDC_TIMER, (uint32_t)freq_hz);
    if (!s_pulses_on) {
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL, MOTOR_LEDC_DUTY_50);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL);
        s_pulses_on = true;
    }
}

// ISR przepelnienia PCNT: licznik HW auto-zeruje sie na progu, my akumulujemy.
static void IRAM_ATTR pcnt_overflow_isr(void *arg)
{
    (void)arg;
    uint32_t status = 0;
    pcnt_get_event_status(MOTOR_PCNT_UNIT, &status);
    portENTER_CRITICAL_ISR(&s_pos_mux);
    if (status & PCNT_EVT_H_LIM) {
        s_pos_accum += MOTOR_PCNT_H_LIM;
    }
    if (status & PCNT_EVT_L_LIM) {
        s_pos_accum -= MOTOR_PCNT_H_LIM;
    }
    portEXIT_CRITICAL_ISR(&s_pos_mux);
}

// Surowa pozycja (akumulator + biezacy licznik HW), bez offsetu zera.
static int32_t pos_raw(void)
{
    int16_t hw = 0;
    int32_t accum;
    portENTER_CRITICAL(&s_pos_mux);
    pcnt_get_counter_value(MOTOR_PCNT_UNIT, &hw);
    accum = s_pos_accum;
    portEXIT_CRITICAL(&s_pos_mux);
    return accum + (int32_t)hw;
}

static void position_counter_init(void)
{
    // DIR steruje kierunkiem zliczania. Chcemy: applied>0 (przod) => pozycja
    // rosnie. Przod = DIR poziom wysoki (chyba ze dir_invert).
    pcnt_count_mode_t hctrl = s_cfg.dir_invert ? PCNT_MODE_REVERSE : PCNT_MODE_KEEP;
    pcnt_count_mode_t lctrl = s_cfg.dir_invert ? PCNT_MODE_KEEP : PCNT_MODE_REVERSE;

    // WAZNE: nie oddajemy pinow PCNT-owi (PCNT_PIN_NOT_USED), bo jego konfiguracja
    // przestawia pin na wejscie i przy okazji odlacza sygnal wyjsciowy LEDC od
    // STEP. Zamiast tego zostawiamy LEDC/GPIO jako wlascicieli pinow, a sygnaly
    // do PCNT podpinamy recznie przez matryce GPIO (loopback).
    pcnt_config_t pc = {
        .pulse_gpio_num = PCNT_PIN_NOT_USED,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .channel = PCNT_CHANNEL_0,
        .unit = MOTOR_PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,   // zliczaj na zboczu narastajacym STEP
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = lctrl,
        .hctrl_mode = hctrl,
        .counter_h_lim = MOTOR_PCNT_H_LIM,
        .counter_l_lim = -MOTOR_PCNT_H_LIM,
    };
    pcnt_unit_config(&pc);

    pcnt_event_enable(MOTOR_PCNT_UNIT, PCNT_EVT_H_LIM);
    pcnt_event_enable(MOTOR_PCNT_UNIT, PCNT_EVT_L_LIM);
    pcnt_counter_pause(MOTOR_PCNT_UNIT);
    pcnt_counter_clear(MOTOR_PCNT_UNIT);
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(MOTOR_PCNT_UNIT, pcnt_overflow_isr, NULL);
    pcnt_counter_resume(MOTOR_PCNT_UNIT);

    // Wlacz bufory wejsciowe na pinach STEP/DIR (sa wyjsciami LEDC/GPIO).
    // UWAGA: na ESP32 wlaczenie wyjscia w gpio_set_direction resetuje zrodlo
    // wyjsciowe pinu do zwyklego GPIO (odlacza LEDC). Dlatego ponizej, po
    // ustawieniu INPUT_OUTPUT, JAWNIE przywracamy sygnal wyjsciowy LEDC na STEP.
    gpio_set_direction((gpio_num_t)s_cfg.step_pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction((gpio_num_t)s_cfg.dir_pin, GPIO_MODE_INPUT_OUTPUT);

    // Podepnij wejscia PCNT do tych samych pinow (loopback przez matryce).
    esp_rom_gpio_connect_in_signal((gpio_num_t)s_cfg.step_pin,
                                   PCNT_SIG_CH0_IN0_IDX, false);
    esp_rom_gpio_connect_in_signal((gpio_num_t)s_cfg.dir_pin,
                                   PCNT_CTRL_CH0_IN0_IDX, false);

    // Przywroc wyjscie LEDC (kanal 0, high-speed) na pin STEP. DIR zostaje
    // zwyklym wyjsciem GPIO (sterowanym przez gpio_set_level).
    esp_rom_gpio_connect_out_signal((gpio_num_t)s_cfg.step_pin,
                                    LEDC_HS_SIG_OUT0_IDX, false, false);
}

void motor_init(const motor_config_t *cfg)
{
    s_cfg = *cfg;
    if (s_cfg.min_speed_hz < 1.0f) {
        s_cfg.min_speed_hz = 1.0f;
    }
    s_acc = s_cfg.default_acc;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_cfg.dir_pin) | (1ULL << s_cfg.en_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    dir_pin_write(true);
    en_pin_write(false);  // domyslnie naped wylaczony (brak auto-enable)

    ledc_timer_config_t tcfg = {
        .speed_mode = MOTOR_LEDC_MODE,
        .timer_num = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_RES,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&tcfg);

    ledc_channel_config_t ccfg = {
        .gpio_num = s_cfg.step_pin,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL,
        .timer_sel = MOTOR_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ledc_channel_config(&ccfg);
    pulses_stop();

    // Licznik pozycji wozka (po konfiguracji LEDC, bo przywraca tryb pinow).
    position_counter_init();
}

void motor_set_speed_hz(float hz)
{
    if (hz > s_cfg.max_speed_hz) {
        hz = s_cfg.max_speed_hz;
    } else if (hz < -s_cfg.max_speed_hz) {
        hz = -s_cfg.max_speed_hz;
    }
    s_target_hz = hz;
}

void motor_set_acc(uint32_t acc)
{
    portENTER_CRITICAL(&s_mux);
    s_acc = acc;
    portEXIT_CRITICAL(&s_mux);
}

void motor_enable(bool en)
{
    portENTER_CRITICAL(&s_mux);
    s_enabled = en;
    portEXIT_CRITICAL(&s_mux);
    en_pin_write(en);
    if (!en) {
        s_target_hz = 0.0f;
        s_applied_hz = 0.0f;
        pulses_stop();
    }
}

void motor_stop(void)
{
    s_target_hz = 0.0f;
    s_applied_hz = 0.0f;
    pulses_stop();
}

void motor_driver_tick(float dt_s)
{
    if (!s_enabled) {
        if (s_applied_hz != 0.0f) {
            s_applied_hz = 0.0f;
        }
        pulses_stop();
        return;
    }

    uint32_t acc;
    portENTER_CRITICAL(&s_mux);
    acc = s_acc;
    portEXIT_CRITICAL(&s_mux);

    float applied = s_applied_hz;
    const float target = s_target_hz;

    if (acc == 0) {
        applied = target;  // bez rampy
    } else {
        const float max_step = (float)acc * dt_s;
        const float diff = target - applied;
        if (diff > max_step) {
            applied += max_step;
        } else if (diff < -max_step) {
            applied -= max_step;
        } else {
            applied = target;
        }
    }

    // Soft-limity pozycji: blokuj ruch "na zewnatrz" poza +/- limit, ale
    // pozwol wracac do srodka. Twardy LIMIT (krancowka) to osobny fault.
    bool limited = false;
    if (s_cfg.soft_limit_steps > 0) {
        const int32_t pos = motor_get_position_steps();
        if (pos >= s_cfg.soft_limit_steps && applied > 0.0f) {
            applied = 0.0f;
            limited = true;
        } else if (pos <= -s_cfg.soft_limit_steps && applied < 0.0f) {
            applied = 0.0f;
            limited = true;
        }
    }
    s_soft_limit_hit = limited;

    portENTER_CRITICAL(&s_mux);
    s_applied_hz = applied;
    portEXIT_CRITICAL(&s_mux);

    dir_pin_write(applied >= 0.0f);
    pulses_set(fabsf(applied));
}

int32_t motor_get_position_steps(void)
{
    int16_t hw = 0;
    int32_t accum, offset;
    portENTER_CRITICAL(&s_pos_mux);
    pcnt_get_counter_value(MOTOR_PCNT_UNIT, &hw);
    accum = s_pos_accum;
    offset = s_pos_offset;
    portEXIT_CRITICAL(&s_pos_mux);
    return accum + (int32_t)hw - offset;
}

void motor_zero_position(void)
{
    const int32_t raw = pos_raw();
    portENTER_CRITICAL(&s_pos_mux);
    s_pos_offset = raw;
    portEXIT_CRITICAL(&s_pos_mux);
}

bool motor_soft_limit_active(void)
{
    return s_soft_limit_hit;
}

float motor_get_applied_speed_hz(void)
{
    float v;
    portENTER_CRITICAL(&s_mux);
    v = s_applied_hz;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

uint32_t motor_get_applied_acc(void)
{
    uint32_t v;
    portENTER_CRITICAL(&s_mux);
    v = s_acc;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

bool motor_is_enabled(void)
{
    bool v;
    portENTER_CRITICAL(&s_mux);
    v = s_enabled;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

void motor_task(void *arg)
{
    QueueHandle_t q = (QueueHandle_t)arg;
    const TickType_t period = pdMS_TO_TICKS(2);  // ~500 Hz rampa
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    bool was_fault = false;

    for (;;) {
        // 1) Konsumpcja komend.
        motor_cmd_t cmd;
        while (xQueueReceive(q, &cmd, 0) == pdTRUE) {
            switch (cmd.type) {
                case MOTOR_CMD_SET_ENABLE:
                    motor_enable(cmd.arg.b);
                    break;
                case MOTOR_CMD_SET_SPEED:
                    motor_set_speed_hz(cmd.arg.f);
                    break;
                case MOTOR_CMD_SET_ACC:
                    motor_set_acc(cmd.arg.u);
                    break;
                case MOTOR_CMD_STOP:
                    motor_set_speed_hz(0.0f);
                    break;
                case MOTOR_CMD_RESET_FAULT:
                    safety_reset_fault();
                    break;
                case MOTOR_CMD_ZERO_POSITION:
                    motor_zero_position();
                    break;
                default:
                    break;
            }
        }

        // 2) Egzekucja blokady przy fault.
        const bool fault = safety_is_fault();
        if (fault) {
            if (!was_fault) {
                motor_enable(false);
                motor_stop();
                was_fault = true;
            }
        } else {
            was_fault = false;
        }

        // 3) Rampa predkosci (tylko gdy brak fault).
        const int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_us) * 1e-6f;
        last_us = now;
        if (dt <= 0.0f) {
            dt = 0.002f;
        }
        if (!fault) {
            motor_driver_tick(dt);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
