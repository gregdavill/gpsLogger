/*
 * Led.h
 *
 *  Created on: 12 Jul 2016
 *      Author: greg
 */

#ifndef LED_H__
#define LED_H__

typedef struct
{
	int port;
	int pin;
}pinLed_t;

class Led
{

public:
	void Init();
};

#endif /* SRC_HAL_PUBLIC_LED_H_ */
