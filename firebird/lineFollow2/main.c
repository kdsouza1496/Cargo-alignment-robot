#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "Motor.h"
#include "lcd.h"
#include "ADC.h"

int getError(void);
void line_follow_mm(int);
void line_follow(void);
void turnLeft();
void takeUturn();
void turnRight();

//LastError keeps track of the previous error. It  is used to calculate black and also kept for use in PID control in the future.
int error = 0;
int lastError = 0;

//Proportional term in the error correction.
int kp = 18;

//This variable detects the number of black nodes and keeps count of it.


int state = 0;

int main()
{
    cli();
    set_motors();
    set_ADC();
    set_lcd();
    sei();

    PORTH |= (1<<3) | (1<<2);

    lcd_cursor(1,4);
    lcd_string("E-Yantra");

    //This delay is introduced to allow the sensors settle down to a stable reading.
    _delay_ms(5000);

    //Uncomment this to check the bot following the line for a particular distance.
    //line_follow_mm(800);

    while(1)
    {

        line_follow();

    }

}


int getError()
{
    int error = 0;
    int count = 0;

    int left, right, centre;
    centre = ADC_Conversion(2);
    right = ADC_Conversion(1);
    left = ADC_Conversion(3);

    lcd_print(2, 1, left, 3);
    lcd_print(2, 5, centre, 3);
    lcd_print(2, 9, right, 3);

    if(centre > 90)
        count++;

    if(left > 20)
    {
        error += 2;
        count++;
    }

    if(right > 25)
    {
        error -= 2;
        count++;
    }

    if(count == 3)
        return 200;

    if(count == 0)
        return -200;

    return error/count;

}


void line_follow_mm(int distance)
{
    int requiredCount;

    velocity(0, 0);
    forward();

    int error = 0;

    ShaftCountLeft = 0;
    requiredCount = distance / 5.338;

    while(ShaftCountLeft < requiredCount)
    {
        error = getError();
        if(error > -100 && error < 100)
        {
            velocity(150 - error, 180 + error);
        }
    }

    stop();
}

void line_follow()
{
    error = getError();

    //Error is 200 if a node is detected.
    if(error == 200)
    {
        if(lastError != error)
        {
               switch(state)
               {
                    case 0 : turnLeft(); break;
                    case 1 : break;
                    case 2 : takeUturn(); break;
                    case 3 : break;
                    case 4 : break;
                    case 5 : break;
                    case 6 : takeUturn(); break;
                    case 7 : break;
                    case 8 : turnRight(); break;
                }

                state++;
                if(state == 9)
                {
                    state = 0;
                }

        }

    }

    //Error is -200 if all sensors are sensing white. This condition is not handled.
    else if(error != -200)
    {
            error = kp * error;
            velocity(150 - error, 180 + error);

            forward();
    }


    lastError = error;
}


void turnLeft()
{
    velocity(150, 180);
    line_follow_mm(85);
    stop();
    _delay_ms(500);
    left_degrees(100);
    stop();
    _delay_ms(500);
}

void takeUturn()
{
    velocity(150, 180);
    stop();
    _delay_ms(500);
    forward_mm(50);
    left_degrees(180);
    stop();
    _delay_ms(500);
}

void turnRight()
{
    velocity(150, 180);
    line_follow_mm(85);
    stop();
    _delay_ms(500);
    right_degrees(100);
    stop();
    _delay_ms(500);
}
