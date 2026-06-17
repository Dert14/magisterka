#include "encoder.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

// Tablica dekodera kwadraturowego: indeks = (prev_state << 2) | curr_state.
// Stan = (A << 1) | B. Wartosc = +/-1 krok lub 0 (brak/niepoprawne przejscie).
static const int8_t kQuadTable[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
};

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int32_t s_count = 0;
static volatile uint8_t s_state = 0;
static int s_pin_a = -1;
static int s_pin_b = -1;

// ISR: tylko odczyt 2 pinow, lookup w tablicy, atomowa aktualizacja licznika.
static void IRAM_ATTR encoder_isr(void *arg)
{
    (void)arg;
    const uint8_t a = (uint8_t)gpio_get_level((gpio_num_t)s_pin_a);
    const uint8_t b = (uint8_t)gpio_get_level((gpio_num_t)s_pin_b);
    const uint8_t state = (uint8_t)((a << 1) | b);
    const uint8_t idx = (uint8_t)((s_state << 2) | state);
    const int8_t delta = kQuadTable[idx];
    s_state = state;

    if (delta != 0) {
        portENTER_CRITICAL_ISR(&s_mux);
        s_count += delta;
        portEXIT_CRITICAL_ISR(&s_mux);
    }
}

void encoder_init(int pin_a, int pin_b)
{
    s_pin_a = pin_a;
    s_pin_b = pin_b;

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pin_a) | (1ULL << pin_b),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io);

    const uint8_t a = (uint8_t)gpio_get_level((gpio_num_t)pin_a);
    const uint8_t b = (uint8_t)gpio_get_level((gpio_num_t)pin_b);
    s_state = (uint8_t)((a << 1) | b);

    gpio_isr_handler_add((gpio_num_t)pin_a, encoder_isr, NULL);
    gpio_isr_handler_add((gpio_num_t)pin_b, encoder_isr, NULL);
}

int32_t encoder_get_count(void)
{
    int32_t v;
    portENTER_CRITICAL(&s_mux);
    v = s_count;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

void encoder_reset(void)
{
    portENTER_CRITICAL(&s_mux);
    s_count = 0;
    portEXIT_CRITICAL(&s_mux);
}
