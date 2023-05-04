/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAMC21 has 9 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3
#define GPIO_PIN_FUNCTION_E 4
#define GPIO_PIN_FUNCTION_F 5
#define GPIO_PIN_FUNCTION_G 6
#define GPIO_PIN_FUNCTION_H 7
#define GPIO_PIN_FUNCTION_I 8

#define PA04 GPIO(GPIO_PORTA, 4)
#define PA05 GPIO(GPIO_PORTA, 5)
#define LEDYellow GPIO(GPIO_PORTA, 15)
#define PA16 GPIO(GPIO_PORTA, 16)
#define CS GPIO(GPIO_PORTA, 17)
#define PA18 GPIO(GPIO_PORTA, 18)
#define PA19 GPIO(GPIO_PORTA, 19)
#define RTD_SWITCH GPIO(GPIO_PORTB, 12)
#define AMS_LIGHT GPIO(GPIO_PORTA, 21)
#define PA24 GPIO(GPIO_PORTA, 24)
#define PA25 GPIO(GPIO_PORTA, 25)
#define TS_BUTTON GPIO(GPIO_PORTB, 5)

#endif // ATMEL_START_PINS_H_INCLUDED
