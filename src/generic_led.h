/*
 * Simple functions to control board LEDs
 */

#ifndef GENERIC_LED__H
#define GENERIC_LED__H

int generic_led_init(void);
void red_led_blink(void);
void red_led_on(void);
void blue_led_blink(void);
void blue_led_on(void);

#endif

