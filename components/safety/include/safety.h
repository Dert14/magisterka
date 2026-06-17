// safety - obsluga krancowki (LIMIT) i zatrzasku bledu (fault latch).
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int limit_pin;
    bool limit_active_high;  // true: zadzialanie krancowki => poziom HIGH
    int period_ms;           // okres cyklicznego odczytu (np. 2 ms = 500 Hz)
    int debounce_ms;         // czas stabilnego stanu do uznania (np. 10 ms)
} safety_config_t;

// Inicjalizacja pinu LIMIT (wejscie). Nie uruchamia jeszcze taska.
void safety_init(const safety_config_t *cfg);

// Task cyklicznego odczytu LIMIT + zatrzask bledu. arg: nieuzywany.
void safety_task(void *arg);

// true jesli aktywny zatrzask bledu (trzyma do safety_reset_fault).
bool safety_is_fault(void);

// Biezacy (chwilowy) stan krancowki.
bool safety_get_limit_state(void);

// Kasuje zatrzask bledu wylacznie gdy krancowka nie jest aktualnie aktywna.
// Zwraca true gdy fault zostal skasowany.
bool safety_reset_fault(void);

#ifdef __cplusplus
}
#endif
