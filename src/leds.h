#ifndef LEDS_H
#define LEDS_H

#include <stdint.h>

void beacon_init(void);
void beacon_step(void);

void hbled_init(void);
void hbled_toggle(int);

void rgb_init(void);
void rgb_set(uint32_t, uint32_t, uint32_t);

void stripe_init(void);
void stripe_set(uint32_t);
void stripe_set_level(int);
void stripe_out(void);
void stripe_scroll();

#endif
