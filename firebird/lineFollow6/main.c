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
void arenaLeft(void);
void arenaRight();
void arenaUturn();
void arenaStraight(void);
int getError(void);
int checkForCube(void);

int kp = 8;
int leftSpeed, rightSpeed;
int centre, sensorLeft, sensorRight;
int state = 0;

int main(void)
{
    cli();
    set_lcd();
    set_motors();
    set_ADC();
    sei();

    PORTH |= (1<<2) | (1 << 3);

    int error = 0, lastError = 0, node = 0;

    forward();
    velocity(0,0);

    while(1)
    {
        error = getError();

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
                switch(state)
                {
                    case 0 : arenaLeft(); state++; break;
                    case 1 : arenaStraight(); state++; break;
                    case 2 : arenaUturn();state++; break;
                    case 3 : arenaStraight(); state++; break;
                    case 4 : arenaStraight(); state++; break;
                    case 5 : arenaStraight(); state++; break;
                    case 6 : arenaUturn(); state++; break;
                    case 7 : arenaStraight(); state++; break;
                    case 8 : arenaRight(); state++;
                }

                if(state == 9)
                {
                    stop();
                    while(1);
                }
            }
        }
        lastError = error;
    }


    return 0;
}

void line_follow_mm(int distance)
{
    int reqCount = distance/5.338;
    int error = 0;

    ShaftCountLeft = 0;
    forward();

    while(ShaftCountLeft < reqCount)
    {
        error = getError();


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

void arenaLeft(void)
{
                stop();
                _delay_ms(1000);
                velocity(150, 180);

                line_follow_mm(60);
                stop();
                _delay_ms(1000);

/*
                soft_left();
                _delay_ms(1000);
*/
                velocity(150, 180);
                left();
                _delay_ms(1000);

                centre = ADC_Conversion(2);

                while(centre < 90)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(1000);

                forward();
}

void arenaRight(void)
{
                stop();
                _delay_ms(1000);
                velocity(150, 180);

                line_follow_mm(60);
                stop();
                _delay_ms(1000);
/*
                soft_right();
                _delay_ms(1000);
*/
                velocity(150, 180);
                right();
                _delay_ms(1000);

                centre = ADC_Conversion(2);

                while(centre < 90)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(1000);

                forward();
}

void arenaUturn(void)
{
                stop();
                _delay_ms(1000);
                velocity(150, 180);

                line_follow_mm(60);
                stop();
                _delay_ms(1000);
/*
                soft_right();
                _delay_ms(1000);
*/
                velocity(150, 180);
                right();
                _delay_ms(2000);

                centre = ADC_Conversion(2);

                while(centre < 90)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(1000);

                forward();
}

void arenaStraight(void)
{
    stop();
    _delay_ms(1000);
    forward();
    velocity(150, 180);
}

int getError(void)
{
    int error;
        centre = ADC_Conversion(2);
        sensorLeft = ADC_Conversion(3);
        sensorRight = ADC_Conversion(1);

        lcd_print(2, 1, sensorLeft, 3);
        lcd_print(2, 5, centre, 3);
        lcd_print(2, 9, sensorRight, 3);

        if(sensorLeft > 10 && sensorRight > 20)
            error = 100;

        else if(centre > 90)
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

        return error;
}

int checkForCube(void)
{
    //turn on the Sharp sensors.
    PORTH &= ~(1 << 3);

    int value = ADC_Conversion(11);

    if(value > 130)
        return 1;

    if(value > 45)
        return 2;

    else
        return 0;
}
