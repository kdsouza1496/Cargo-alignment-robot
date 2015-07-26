#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "ADC.h"
#include "lcd.h"

#define leftIncrease 20
#define rightIncrease 20

void line_follow_mm(int);


int kp = 8;
int error = 0, leftSpeed, rightSpeed, lastError = 0, node = 0;
int centre, sensorLeft, sensorRight;


int main(void)
{
    cli();
    set_lcd();
    set_motors();
    set_ADC();
    sei();

    PORTH |= (1<<3) | (1<<2);


    forward();
    velocity(0,0);


    while(1)
    {
        centre = ADC_Conversion(2);
        sensorLeft = ADC_Conversion(3);
        sensorRight = ADC_Conversion(1);

        lcd_print(2, 1, sensorLeft, 3);
        lcd_print(2, 5, centre, 3);
        lcd_print(2, 9, sensorRight, 3);

        if(centre > 115 && sensorLeft > 15 && sensorRight > 20)
            error = 100;

        else if(centre > 115)
            error = 0;

        else if(sensorLeft > 45)
            error = 7;

        else if(sensorLeft > 25)
            error = 6;

        else if(sensorLeft > 17)
            error = 5;

        else if(sensorLeft > 9)
            error = 4;

        else if(sensorRight > 110)
            error = -7;

        else if(sensorRight > 85)
            error = -6;

        else if(sensorRight > 55)
            error = -5;

        else if(sensorRight > 35)
            error = -4;

        else if(sensorRight > 25)
            error = -3;

        else if(sensorRight > 18)
            error = -2;

        else if(sensorRight > 15)
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
                _delay_ms(1000);
                velocity(150, 180);

                line_follow_mm(80);
                stop();
                _delay_ms(2000);

                left();
                _delay_ms(1000);

                centre = ADC_Conversion(2);

                while(centre < 120)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(2000);

                forward();
            }

            if(node == 4)
            {
                _delay_ms(1000);
                velocity(150, 180);

                line_follow_mm(80);
                stop();
                _delay_ms(2000);

                right();
                _delay_ms(1000);

                centre = ADC_Conversion(2);

                while(centre < 120)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(2000);

                forward();
            }

            if(node == 5)
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




void line_follow_mm(int distance)
{
    int reqCount = distance/5.338;

    ShaftCountLeft = 0;
    forward();

    while(ShaftCountLeft < reqCount)
    {
        centre = ADC_Conversion(2);
        sensorLeft = ADC_Conversion(3);
        sensorRight = ADC_Conversion(1);

        lcd_print(2, 1, sensorLeft, 3);
        lcd_print(2, 5, centre, 3);
        lcd_print(2, 9, sensorRight, 3);

        if(centre > 115 && sensorLeft > 15 && sensorRight > 20)
            error = 100;

        else if(centre > 115)
            error = 0;

        else if(sensorLeft > 45)
            error = 7;

        else if(sensorLeft > 25)
            error = 6;

        else if(sensorLeft > 17)
            error = 5;

        else if(sensorLeft > 9)
            error = 4;

        else if(sensorRight > 110)
            error = -7;

        else if(sensorRight > 85)
            error = -6;

        else if(sensorRight > 55)
            error = -5;

        else if(sensorRight > 35)
            error = -4;

        else if(sensorRight > 25)
            error = -3;

        else if(sensorRight > 18)
            error = -2;

        else if(sensorRight > 15)
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
    }
}
