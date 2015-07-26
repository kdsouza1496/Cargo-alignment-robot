/*
 */

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "Motor.h"

int main(void)
{

    cli();
    set_motors();
    sei();
    // Insert code
    velocity(150, 182);
    forward_mm(800);
    while(1)
    {

    }

    return 0;
}
