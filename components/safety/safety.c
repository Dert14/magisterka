#include "safety.h"

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool s_fault = false;
static volatile bool s_limit = false;
static safety_config_t s_cfg;

void safety_init(const safety_config_t *cfg)
{
    s_cfg = *cfg;
    if (s_cfg.period_ms <= 0) {
        s_cfg.period_ms = 2;
    }
    if (s_cfg.debounce_ms < 0) {
        s_cfg.debounce_ms = 0;
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << s_cfg.limit_pin),
        .mode = GPIO_MODE_INPUT,
        // Topologia: krancowki NC szeregowo do masy + zewnetrzny pull-up.
        // Wewnetrzny PULL-UP (rownolegle do zewnetrznego) definiuje twarde HIGH
        // przy zadzialaniu (NC rozwarte / urwany przewod = fail-safe), a zwarcie
        // do masy w spoczynku i tak wygrywa => twarde LOW. Zapobiega plywaniu.
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

static bool read_limit_active(void)
{
    const int lvl = gpio_get_level((gpio_num_t)s_cfg.limit_pin);
    return s_cfg.limit_active_high ? (lvl != 0) : (lvl == 0);
}

void safety_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(s_cfg.period_ms);
    TickType_t last = xTaskGetTickCount();

    // Debounce: stan musi byc stabilny przez >= deb_needed probek, zanim go
    // uznamy. Odrzuca drgania stykow i krotkie szpilki.
    int deb_needed = s_cfg.debounce_ms / s_cfg.period_ms;
    if (deb_needed < 1) {
        deb_needed = 1;
    }

    bool confirmed = read_limit_active();
    int cnt = 0;

    for (;;) {
        const bool raw = read_limit_active();
        if (raw == confirmed) {
            cnt = 0;  // stan zgodny - reset licznika zmiany
        } else if (++cnt >= deb_needed) {
            confirmed = raw;  // zmiana utrzymana przez deb_needed probek
            cnt = 0;
        }

        portENTER_CRITICAL(&s_mux);
        s_limit = confirmed;
        if (confirmed) {
            s_fault = true;  // zatrzask
        }
        portEXIT_CRITICAL(&s_mux);

        vTaskDelayUntil(&last, period);
    }
}

bool safety_is_fault(void)
{
    bool v;
    portENTER_CRITICAL(&s_mux);
    v = s_fault;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

bool safety_get_limit_state(void)
{
    bool v;
    portENTER_CRITICAL(&s_mux);
    v = s_limit;
    portEXIT_CRITICAL(&s_mux);
    return v;
}

bool safety_reset_fault(void)
{
    bool cleared = false;
    portENTER_CRITICAL(&s_mux);
    if (!s_limit) {
        s_fault = false;
        cleared = true;
    }
    portEXIT_CRITICAL(&s_mux);
    return cleared;
}
