// motor - warstwa abstrakcji napedu (motor_driver) + task wykonawczy.
//
// Backend v1: generacja stalej czestotliwosci STEP przez LEDC (square wave),
// kierunek przez DIR, software'owy EN. API jest stabilne pod podmiane backendu
// (RMT / timer pulse / sterownik krokowy) bez zmian w warstwie wyzszej.
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int step_pin;
    int dir_pin;
    int en_pin;
    bool en_active_high;   // poziom aktywny pinu EN (false = aktywny LOW)
    bool dir_invert;       // odwrocenie logiki kierunku
    float max_speed_hz;    // ograniczenie |predkosci|
    float min_speed_hz;    // ponizej tej |predkosci| pulsy sa wylaczane
    uint32_t default_acc;  // domyslne przyspieszenie [Hz/s]
    int32_t soft_limit_steps;  // +/- limit pozycji wozka (0 = wylaczony)
} motor_config_t;

// ---- API motor_driver (warstwa abstrakcji) ----
void motor_init(const motor_config_t *cfg);
void motor_set_speed_hz(float hz);  // zadana predkosc (znak = kierunek)
void motor_set_acc(uint32_t acc);   // przyspieszenie rampy [Hz/s]
void motor_enable(bool en);         // software'owy EN
void motor_stop(void);              // natychmiastowy stop (predkosc -> 0)

// Krok rampy + aplikacja na sprzet. Wolane cyklicznie przez motor_task.
void motor_driver_tick(float dt_s);

// Bezpieczne (atomowe) odczyty stanu dla telemetrii.
float motor_get_applied_speed_hz(void);
uint32_t motor_get_applied_acc(void);
bool motor_is_enabled(void);

// Pozycja wozka w krokach (PCNT, licznik 32-bit signed, znak = kierunek).
int32_t motor_get_position_steps(void);

// Ustawia biezaca pozycje jako zero (homing wykonuje warstwa wyzsza/PC).
void motor_zero_position(void);

// 1 gdy ostatni tick zostal ograniczony soft-limitem pozycji.
bool motor_soft_limit_active(void);

// ---- Kolejka komend / task ----
typedef enum {
    MOTOR_CMD_SET_ENABLE,
    MOTOR_CMD_SET_SPEED,
    MOTOR_CMD_SET_ACC,
    MOTOR_CMD_STOP,
    MOTOR_CMD_RESET_FAULT,
    MOTOR_CMD_ZERO_POSITION,
} motor_cmd_type_t;

typedef struct {
    motor_cmd_type_t type;
    union {
        bool b;
        float f;
        uint32_t u;
    } arg;
} motor_cmd_t;

// Task: konsumuje komendy z kolejki (arg = QueueHandle_t motor_cmd_t),
// egzekwuje blokade przy fault, prowadzi rampe predkosci.
void motor_task(void *arg);

#ifdef __cplusplus
}
#endif
