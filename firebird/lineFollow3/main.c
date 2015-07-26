/*
 */
#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "lcd.h"
#include "ADC.h"

int getError(void);

int kp = 15;
float incRate = 10;

int main(void)
{
    cli();
    set_ADC();
    set_motors();
    set_lcd();
    sei();

    int error = 0;
    velocity(0, 0);
    forward();
    int leftSpeed, rightSpeed;

    PORTH |= (1<<2) | (1<<3);

    _delay_ms(3000);

    while(1)
    {
        error = getError();

        if(error > -50 && error < 50)
        {
            rightSpeed = 180 + error*kp + error*incRate;
            leftSpeed = 150 - error*kp;

            if(leftSpeed < 0)
                leftSpeed = 0;
            if(leftSpeed >255)
                leftSpeed = 255;
            if(rightSpeed < 0)
                rightSpeed = 0;
            if(rightSpeed > 255)
                rightSpeed = 255;

            velocity(leftSpeed, rightSpeed);
        }

    }

    return 0;
}

int getError()
{

    int left, right, centre;

    centre = ADC_Conversion(2);
    right = ADC_Conversion(1);
    left = ADC_Conversion(3);

    lcd_print(2, 1, left, 3);
    lcd_print(2, 5, centre, 3);
    lcd_print(2, 9, right, 3);

    if(centre > 100 && left > 25 && right > 30)
        return 200;

    if(centre >= 120)
        return 0;

    if(right - 20 > 0)
        return -(right - 20)/95*10;

    if(left - 12 > 0)
        return (left - 12)/31*10;

    return -200;

}
