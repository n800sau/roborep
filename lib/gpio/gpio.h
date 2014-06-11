/* 
 * https://github.com/mrshu/GPIOlib
 * Copyright (c) 2011, Copyright (c) 2011 mr.Shu
 * All rights reserved. 
 * 
 * Modified on 24 June 2012, 11:06 AM
 * File:   gpio.h
 * Author: purinda (purinda@gmail.com)
 * 
 */

#ifndef GPIO_H
#define	GPIO_H

#define GPIO_LOW 0
#define GPIO_HIGH 1

void digitalWrite(int pin, int value);
int digitalRead(int pin);

#endif	/* GPIO_H */

