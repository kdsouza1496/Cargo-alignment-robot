/****************************************************************************************
Author : SYED MOHAMMED YOUSUFF HUSSAIN
Date : 1/12/2014.
Tested good.
***************************************************************************************/

/*
* The values below were taken in normal room lighting conditions.
* The front sensor gives a minimum reading of 70 at a distance of 3 cm and a max reading of 160 at a distance of 14 cm.
* All other sensors give a minimum reading of 25-60 at a distance of 1 cm and a maximum reading of 160.
*/

#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lcd.h"
#include "ADC.h"


int main(void)
{
    cli();
    set_ADC();
    set_lcd();
    sei();

    PORTH |= 0x08; //Turn off the IR sharp sensors.
    PORTG |= 0x04; //Turn off the White line sensors.

    int sensors[5];
    int i;

    while(1)
    {
        for(i=0;i<5;i++)
        {
            sensors[i] = ADC_Conversion(i+4);
            if(i==2)
                lcd_print(1, 7, sensors[i], 4);
            else if(i < 2)
                lcd_print(2, 4*(i) + 1, sensors[i], 3);
            else
                lcd_print(2, 4*(i-1) + 2, sensors[i], 3);
        }

    }

    return 0;
}
