#ifndef GPIO_H
#define	GPIO_H

#define GPIO_LOW 0
#define GPIO_HIGH 1

void digitalWrite(int pin, int value);
int digitalRead(int pin);
void wait_interrupt(int pin);

#endif	/* GPIO_H */

