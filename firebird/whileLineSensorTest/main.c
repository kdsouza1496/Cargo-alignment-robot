#define F_CPU 14745600

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Motor.h"
#include "lcd.h"
#include "ADC.h"

int kp = 15;

int main()
{
    cli();
    set_motors();
    set_ADC();
    set_lcd();
    sei();

    //Turn off the proximity sensors and the sharp sensor to save power.
    PORTH |= (1<<3) | (1<<2);

    int left, right, centre;
    while(1)
    {
        left = ADC_Conversion(3);
        centre = ADC_Conversion(2);
        right = ADC_Conversion(1);

        lcd_print(1, 1, left, 3);
        lcd_print(1, 6, centre, 3);
        lcd_print(1, 11, right, 3);

    }

}




