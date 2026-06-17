// encoder - kwadraturowy odczyt A/B na przerwaniach GPIO.
// Licznik 32-bit signed, lekki ISR + atomowy snapshot.
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Inicjalizacja pinow A/B (wejscia z pull-up, przerwanie na obu zboczach).
// Wymaga wczesniej zainstalowanego serwisu ISR GPIO (gpio_install_isr_service).
void encoder_init(int pin_a, int pin_b);

// Atomowy odczyt biezacego licznika.
int32_t encoder_get_count(void);

// Zerowanie licznika.
void encoder_reset(void);

#ifdef __cplusplus
}
#endif
