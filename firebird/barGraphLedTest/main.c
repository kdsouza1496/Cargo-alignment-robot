/**********************************************************************************
Author : SYED MOHAMMED YOUSUFF HUSSAIN
Date : 1/12/2014
Tested good.
***********************************************************************************/

/*
* Note: Only 8 out of 10 led's of the bar graph are connected to the micro controller.
* All LED's are connected to PORTJ
* To use PORTJ for other purposes, disable the bar graph by removing the jumper.
*/

#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


int main(void)
{
    PORTH |= (1<<2) | (1<<3);  //Turn off proximity and sharp sensors.
    PORTG |= (1<<2);           //Turn off white line sensors.
    int i;

    while(1)
    {
        for(i=0;i<8;i++)
        {
            PORTJ |= (1<<i);
            _delay_ms(50);
        }

        for(i=0;i<8;i++)
        {
            PORTJ &= ~(1<<i);
            _delay_ms(50);
        }
    }

    return 0;
}
