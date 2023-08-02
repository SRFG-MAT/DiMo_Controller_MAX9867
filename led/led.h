/*
 * led.h
 *
 *  Created on: 25.07.2023
 *      Author: hrieser
 */

#ifndef LED_LED_H_
#define LED_LED_H_

#include "cyhal_gpio.h"

cy_rslt_t initLed();
void setRed(bool red);
void setGreen(bool green);
void setStatus(bool red, bool green);
void toggleRed();
void toggleGreen();

#endif /* LED_LED_H_ */
