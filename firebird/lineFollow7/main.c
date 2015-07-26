#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "ADC.h"
#include "lcd.h"

#define forwardLeftSpeed 180
#define forwardRightSpeed 150

void line_follow_mm(int);
void arenaLeft(void);
void arenaLeft_2(void);
void arenaRight();
void arenaUturn();
void arenaStraight(void);
int getError(void);
int getError2(void);
int checkForCube(void);

int kp = 4;
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

    int error = 0, lastError = 0;

    forward();
    velocity(0,0);

    while(1)
    {
        error = getError();

        if(error > -100 && error < 100)
        {
            leftSpeed = forwardLeftSpeed - error*kp;
            rightSpeed = forwardRightSpeed + error*kp;

            velocity(leftSpeed, rightSpeed);
        }
/*
        else if (error == 100)
        {
            if(lastError != error)
            {
                switch(state)
                {
                    case 0 : arenaLeft();
                             state++;
                             switch(checkForCube())
                             {
                                 case 1 : arenaUturn(); state = 5;
                                          switch(checkForCube())
                                          {
                                                case 1 : arenaLeft_2(); state = 9; break;
                                                case 2 : arenaLeft_2(); state = 9; break;
                                                case 0 : break;
                                          }
                                          break;

                                 case 2 : arenaUturn(); state = 5;
                                          switch(checkForCube())
                                          {
                                                case 1 : arenaLeft_2(); state = 9; break;
                                                case 2 : arenaLeft_2(); state = 9; break;
                                                case 0 : break;
                                          }
                                          break;

                                 case 0 : break;
                             }
                             forward();
                             break;

                    case 1 : arenaStraight(); forward(); state++; break;

                    case 2 : arenaUturn();forward(); state++; break;

                    case 3 : arenaStraight(); forward(); state++; break;

                    case 4 : arenaStraight();
                             state++;
                             switch(checkForCube())
                             {
                                 case 1 : arenaLeft(); state = 9; break;
                                 case 2 : arenaLeft(); state = 9; break;
                                 case 0 : break;
                             }
                             forward();
                             break;

                    case 5 : arenaStraight(); forward();state++; break;

                    case 6 : arenaUturn(); forward(); state++; break;

                    case 7 : arenaStraight(); forward();state++; break;

                    case 8 : arenaRight(); forward();  state++;
                }

                if(state == 9)
                {
                    stop();
                    while(1);
                }
            }
        }
*/
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
            leftSpeed = forwardLeftSpeed - error*kp;
            rightSpeed = forwardRightSpeed + error*kp;

            velocity(leftSpeed, rightSpeed);
        }
    }
}

void arenaLeft(void)
{
                stop();
                _delay_ms(1000);
                velocity(forwardLeftSpeed, forwardRightSpeed);

                line_follow_mm(60);
                stop();
                _delay_ms(1000);

/*
                soft_left();
                _delay_ms(1000);
*/
                velocity(forwardLeftSpeed, forwardRightSpeed);
                left();
                _delay_ms(1000);

                centre = ADC_Conversion(2);

                while(centre < 15)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(1000);


}

void arenaLeft_2()
{
                stop();
                _delay_ms(1000);

/*
                soft_left();
                _delay_ms(1000);
*/
                velocity(forwardLeftSpeed, forwardRightSpeed);
                left();
                _delay_ms(1000);

                centre = ADC_Conversion(2);

                while(centre < 15)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(1000);


}

void arenaRight(void)
{
                stop();
                _delay_ms(1000);
                velocity(forwardLeftSpeed, forwardRightSpeed);

                line_follow_mm(60);
                stop();
                _delay_ms(1000);
/*
                soft_right();
                _delay_ms(1000);
*/
                velocity(forwardLeftSpeed, forwardRightSpeed);
                right();
                _delay_ms(1000);

                centre = ADC_Conversion(2);

                while(centre < 15)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(1000);

}

void arenaUturn(void)
{
                stop();
                _delay_ms(1000);
                velocity(forwardLeftSpeed, forwardRightSpeed);
/*
                line_follow_mm(60);
                stop();
                _delay_ms(1000);

                soft_right();
                _delay_ms(1000);
*/
                velocity(forwardLeftSpeed, forwardRightSpeed);
                right();
                _delay_ms(2500);

                centre = ADC_Conversion(2);

                while(centre < 15)
                {
                    centre = ADC_Conversion(2);
                }

                stop();
                _delay_ms(1000);


}

void arenaStraight(void)
{
    stop();
    _delay_ms(1000);

    velocity(forwardLeftSpeed, forwardRightSpeed);
}

int getError2(void)
{
        int error;
        centre = ADC_Conversion(2);
        sensorLeft = ADC_Conversion(3);
        sensorRight = ADC_Conversion(1);

        lcd_print(2, 1, sensorLeft, 3);
        lcd_print(2, 5, centre, 3);
        lcd_print(2, 9, sensorRight, 3);

        if(sensorLeft > 40 && sensorRight > 40)
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

        return -error;
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

        if(sensorLeft > 30 && sensorRight > 30 && centre > 30)
            error = 100;

        else if(centre > 30)
            error = 0;

        else if(sensorLeft > 75)
            error = 4;

        else if(sensorLeft > 50)
            error = 3;

        else if(sensorLeft > 11)
            error = 2;

        else if(sensorLeft > 9)
            error = 1;

        else if(sensorRight > 80)
            error = -4;

        else if(sensorRight > 35)
            error = -3;

        else if(sensorRight > 10)
            error = -2;

        else
            error = -100;

        return error;
}

int checkForCube(void)
{


    int value = ADC_Conversion(11);

    if(value > 130)
        return 1;

    if(value > 45)
        return 2;

    else
        return 0;
}
