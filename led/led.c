/*
 * led.c
 *
 *  Created on: 25.07.2023
 *      Author: hrieser
 */

#include "led.h"
#include "cyhal_gpio.h"
#include "cycfg_pins.h"

cy_rslt_t initLed() {
	cy_rslt_t result;
    result = cyhal_gpio_init(LED_GREEN, CYHAL_GPIO_DIR_OUTPUT,
    		CYHAL_GPIO_DRIVE_STRONG, 0);

    if (result == CY_RSLT_SUCCESS) {
		result = cyhal_gpio_init(LED_RED, CYHAL_GPIO_DIR_OUTPUT,
								 CYHAL_GPIO_DRIVE_STRONG, 0);
    }

    return result;
}

void setRed(bool red) {
	cyhal_gpio_write(LED_RED, red);
}

void setGreen(bool green) {
	cyhal_gpio_write(LED_GREEN, green);
}

void setStatus(bool red, bool green) {
	setRed(red);
	setGreen(green);
}

void toggleRed() {
	cyhal_gpio_toggle(LED_RED);
}

void toggleGreen() {
	cyhal_gpio_toggle(LED_GREEN);
}




