#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Motor.h"

#define forwardLeftSpeed 180
#define forwardRightSpeed 150

#define backLeftSpeed 190
#define backRightSpeed 155

#define leftLeftSpeed 180
#define leftRightSpeed 160

#define rightLeftSpeed 170
#define rightRightSpeed 155



int main()
{
    //Turn off the IR proximity sensors and the IR sharp sensor.
    DDRH |= (1 << 2) | (1 << 3);
    PORTH |= (1 << 2) | (1 << 3);

    //Turn off the white line sensors.
    DDRG |= (1 << 2);
    PORTG |= (1 << 2);

    set_motors();

    while(1)
    {
        velocity(forwardLeftSpeed, forwardRightSpeed);
        forward_mm(1000);

        stop();
        _delay_ms(3000);

        velocity(backLeftSpeed, backRightSpeed);
        back_mm(1000);

        stop();
        _delay_ms(3000);

        velocity(leftLeftSpeed, leftRightSpeed);
        left_degrees(90);

        stop();
        _delay_ms(3000);

        velocity(rightLeftSpeed, rightRightSpeed);
        right_degrees(90);

        stop();
        _delay_ms(3000);

        right_degrees(90);

        stop();
        _delay_ms(3000);

        velocity(leftLeftSpeed, leftRightSpeed);
        left_degrees(90);

        stop();
        _delay_ms(3000);
    }
}
