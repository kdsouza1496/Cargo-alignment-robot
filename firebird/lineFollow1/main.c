/*********************************************************************************************************
DATE : 29th Nov, 2014.
This program is not yet finalized. Changes are to be made as described in the improvements section.
**********************************************************************************************************/

/*
********************* This PROGRAM HAS THE FOLLOWING FEATURES.*********************************************
* 1. It has a function that makes the bot follow the straight line for a fixed amount of distance. (Tested good)
* 2. It has a function that makes the bot follow a straight line and detect nodes. This program will make the bot
*    stop for about a second at the first 6 nodes and at the 7th node, the bot will make a complete halt. (Tested good)
* 3. This program implements only the proportional control and does not have PID.
*/

/*
****************************** IMPROVEMENTS TO BE MADE.**********************************
* 1. Handle the condition when all sensors sense white.
*/

/*
***********************************FURTHER STEPS.*****************************************
* 1. Implement accurate 90 degree turns.
* 2. Implement accurate U-Turns.
* 3. Implement the stage concept to brute force traverse the arena.
* 4. Use the analog value from the sensors to increase precision of travel.
* 5. Implement a Differential control for the line following algorithm.
* 6. Implement an Integral control for the line following algorithm.
*/


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


//LastError keeps track of the previous error. It  is used to calculate black and also kept for use in PID control in the future.
int error = 0;
int lastError = 0;

//Proportional term in the error correction.
int kp = 15;

//This variable detects the number of black nodes and keeps count of it.
int node = 0;

int main()
{
    cli();
    set_motors();
    set_ADC();
    set_lcd();
    sei();

    PORTH |= (1<<3) | (1<<2);

    //This delay is introduced to allow the sensors settle down to a stable reading.

    //Uncomment this to check the bot following the line for a particular distance.
    //line_follow_mm(800);
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




