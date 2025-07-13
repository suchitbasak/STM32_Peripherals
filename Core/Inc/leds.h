/* header for LED-related functions*/

#ifndef LEDS_H
#define LEDS_H

void led_onboard_init(void);
void led_onboard_toggle(void);

void led_debug_init(void);
void led_debug_on(void);
void led_debug_toggle(void);

#endif