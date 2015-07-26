/***********************************************************************************
Author : Syed Mohammed Yousuff Hussain
Date : 1/12/2014.
Tested good.
***********************************************************************************/

/*
****************************************************************************************************
* The IR range sensors give a linear reading between 38(farthest) and 158(Nearest).
* Reading of 32 recorded at 62 cm. This reading is unreliable to calculate the distance but can be used to detect the cube from distance.
* Blind spot distance(Minimum correctly measured distance , reading 158) : 7 cm.
* Reading falls below this distance. Unreliable data.
* Linear reading upto (Maximum correctly measured distance , reading 38 ) : 25 cm
* Reading decreases exponentially slow after this distance. Unreliable data.
*****************************************************************************************************
*/

/*
******************************NOTE********************************************************************************
If jumper J1 is not in place, the pin PH3 is used to software control the power to all the Sharp sensors.
PH3 : Logic 1 : Sensors off.
      Logic 0 : Sensors on.

The same pin is used to software control the power to all the IR proximity sensors. Same logic is used.
*******************************************************************************************************************/


#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "ADC.h"
#include "lcd.h"




int main(void)
{

    cli();
    set_ADC();
    set_lcd();
    sei();

    //Comment the appropriate lines to switch on the sensor.
    PORTG |= 0x04;          //White line sensors turned off.
    PORTH |= 0x04;          //IR sensor turned off.
    //PORTH |= 0x08;          //Sharp sensor turned off.

    int reading;


    while(1)
    {
        reading = ADC_Conversion(11);
        lcd_print(1, 4, reading, 3);


    }

    return 0;
}
