#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "ADC.h"
#include "lcd.h"

#define leftIncrease 20
#define rightIncrease 20

int main(void)
{
    int error = 0, leftSpeed, rightSpeed, lastError = 0, node = 0;
    int kp = 8;
    cli();
    set_lcd();
    set_motors();
    set_ADC();
    sei();

    PORTH |= (1<<3) | (1<<2);
    int centre, left, right;

    forward();
    velocity(0,0);

    while(1)
    {
        centre = ADC_Conversion(2);
        left = ADC_Conversion(3);
        right = ADC_Conversion(1);

        lcd_print(2, 1, left, 3);
        lcd_print(2, 5, centre, 3);
        lcd_print(2, 9, right, 3);

        if(centre > 115 && left > 15 && right > 20)
            error = 100;

        else if(centre > 115)
            error = 0;

        else if(left > 45)
            error = 7;

        else if(left > 25)
            error = 6;

        else if(left > 17)
            error = 5;

        else if(left > 9)
            error = 4;

        else if(right > 110)
            error = -7;

        else if(right > 85)
            error = -6;

        else if(right > 55)
            error = -5;

        else if(right > 35)
            error = -4;

        else if(right > 25)
            error = -3;

        else if(right > 18)
            error = -2;

        else if(right > 15)
            error = -1;

        else
            error = -100;


        if(error > -100 && error < 100)
        {
            leftSpeed = 150 - error*kp;
            rightSpeed = 180 + error*kp;

            if(error>0)
            {
                rightSpeed += 10;
            }
            velocity(leftSpeed, rightSpeed);
        }

        else if (error == 100)
        {
            if(lastError != error)
            {
                stop();
                _delay_ms(1000);

                node++;
            }

            if(node == 3)
            {
                _delay_ms(3000);
            }

            if(node == 6)
            {
                stop();
                while(1);
            }

            else
            {
                forward();
            }

        }

        lastError = error;


    }

    return 0;
}
