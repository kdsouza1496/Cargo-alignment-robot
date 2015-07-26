#define F_CPU 14745600

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "ADC.h"
#include "lcd.h"
#include "myServo.h"

#define forwardLeftSpeed 180        //Calibrated PWM value for the left motor to go straight.
#define forwardRightSpeed 180       //Calibrated PWM value for the right motor to go straight.

void line_follow_mm(int);           //To follow the line for a particular distance. Used while taking 90 degree turns in the arena.
void arenaLeft(void);               //Function to turn left in the arena.
void arenaLeft_2(void);             //Same as arenaLeft, but this function is used when the bot is already on its point of rotation.

void arenaRight();                  //Function to turn right in the arena.
void arenaRight_2();

void arenaUturn();                  //Same as arenaRight, but this function is used when the bot is already on its point of rotation.
void arenaStraight(void);           //Function to continue straight at a node.
int getError(void);                 //This function returns the error of the position of the bot w.r.t the line. Error range : -4 to 4
int checkForCube(void);             //This function checks for the presence of the cube. Returns 1 if cube found and 0 if not.

void buzzerBeep_2();                  //Beeps the buzzer as and when required.
void buzzerBeep_1();

void findLine(void);                //Searches for the line and puts the bot back on it.
void lineFollowBack();              //To follow the line in backward direction.
void skipNodes(int);
void arenaLeft_3();
void arenaRight_3();
int wallFollow();

void buzzer_1();
void buzzer_2();
void buzzer_3();

int isAlignedCorrect_1();
int isAlignedCorrect_2();
int getSharp(void);

void reachEnd();
void arenaRight_4(void);
void checkCubeBehindLeft(void);
void checkCubeBehindRight(void);

//Variables for both d1 and d2
int x = 15;
int k = 40;
int kp = 17;
int row, state, leftSpeed, rightSpeed, centre, sensorLeft, sensorRight, errorFlag;
int d1Completed, d2Completed;
int a[6];
int i;
int cubeBehindLeft=0,cubeBehindRight=0;


int main(void)
{
    int error = 0, lastError = 0;


    cli();
    set_lcd();
    set_motors();
    set_ADC();
    set_servo();
    sei();

    faceFront();
    //_delay_ms(1000);

    ungrip();
    _delay_ms(500);

    DDRJ = 0xFF;
    DDRL |= (1 << 6);   //External buzzer.
    DDRL|= (1 << 7);   //white line sensors

    //D1 starts from here.
    /*********D1 initializations****************/
    state = 0;
    row = 6;
    errorFlag = 0;
    d1Completed = 0;


    velocity(forwardLeftSpeed, forwardRightSpeed);
    line_follow_mm(30);
    skipNodes(1);

    line_follow_mm(30);

    forward();
    while(!d1Completed)
    {
        error = getError();

        if(error == 0)
        {
            leftSpeed = forwardLeftSpeed;
            rightSpeed = forwardRightSpeed;
        }

        else if(error == 1)
        {
            leftSpeed = forwardLeftSpeed + 20;
            rightSpeed = forwardRightSpeed - 20;
        }

        else if(error == -1)
        {
            leftSpeed = forwardLeftSpeed - 20;
            rightSpeed = forwardRightSpeed + 20;
        }

        else if(error == 100)
        {
            if(lastError != error)
            {
                nodeMissTracker = 0;

                switch(state)
                {
                       case 0 : if(row == 0)
                                {
                                    arenaRight();
                                    grip_2();
                                    _delay_ms(500);
                                     line_follow_mm(20);
                                    skipNodes(2);

                                    if(cubeBehindLeft=1)
                                    {
                                        soft_right();
                                        ungrip_2();
                                        //checkalignment from the side;
                                        soft_left();
                                    }
                                    //errorFlag = 0;


                                    errorFlag = 0;

                                    nodeMissedFlag = 0;
                                    nodeMissTracker = 0;



                                    stop();

                                    buzzer_2();

                                    d1Completed = 1;
                                }

                                else if(row == 2)
                                {
                                    arenaRight();

                                    switch(checkForCube())
                                    {
                                        case 1 :state = 4;

                                                //

                                                checkCubeBehindRight();

                                                buzzerBeep_1(); arenaLeft_2(); state = 9; break;

                                                //case 2 : buzzerBeep(); arenaLeft_2(); state = 9; break;
                                        case 2 : state = 5; break;
                                                //case 0 : break;



                                        case 0 :  if(cubeBehindRight=1)
                                                {
                                                    //code here
                                                }
                                                else
                                                {


                                                    arenaLeft_2(); state = 9; break;

                                                }


                                    }
                                }

                                else
                                {
                                    arenaLeft();
                                    switch(checkForCube())
                                    {
                                        case 1 :


                                                // code goes here


                                                checkCubeBehindLeft();



                                                buzzerBeep_1();
                                                arenaUturn();
                                                state = 4;

                                                switch(checkForCube())
                                                {
                                                    case 1 :
                                                             if(row == 1)
                                                             {

                                                                buzzerBeep_1();

                                                                arenaRight_2(); state = 9; break;
                                                             }
                                                             else
                                                             {
                                                                buzzerBeep_1(); arenaLeft_2(); state = 9; break;
                                                             }
                                                //case 2 : buzzerBeep(); arenaLeft_2(); state = 9; break;
                                                    case 2 : state = 5; break;
                                                //case 0 : break;
                                                    case 0 : if(row == 1)
                                                             {
                                                                arenaRight_2(); state = 9; break;
                                                             }
                                                             else
                                                             {
                                                                arenaLeft_2(); state = 9; break;
                                                             }

                                                    }
                                                    break;

                                    case 2 : state = 1; break;

                                    case 0 : state = 4;
                                             arenaUturn();


                                             switch(checkForCube())
                                             {
                                                case 1 : line_follow_mm(6);
                                                         stop();
                                                         if(row == 1)
                                                         {
                                                             buzzerBeep_1(); arenaRight_2(); state = 9; break;
                                                         }
                                                        else
                                                        {

                                                            buzzerBeep_1();

                                                            arenaLeft_2(); state = 9; break;
                                                        }
                                                //case 2 : buzzerBeep(); arenaLeft_2(); state = 9; break;
                                                case 2 : state = 5; break;
                                                //case 0 : break;
                                                case 0 : if(row == 1)
                                                         {
                                                             arenaRight_2(); state = 9; break;
                                                         }
                                                         else
                                                         {
                                                             arenaLeft_2(); state = 9; break;
                                                         }

                                                }
                                                break;
                                    }

                                }
                                forward();
                                break;

                        //case 1 : arenaStraight(); forward(); state++; break;

                        case 1 : line_follow_mm(66);
                                 stop();

                                 buzzerBeep_1();
                                 arenaUturn();
                                 state = 4;

                                //findLine();
                                //stop();
/*
                                 switch(checkForCube())
                                 {
                                     case 1 : buzzerBeep();
                                              arenaUturn();
                                              state = 4; break;

                                     case 2 : arenaUturn();
                                              state = 4;
                                              break;

                                     case 0 : arenaUturn();
                                              state = 4;
                                              break;
                                 }
*/
                                 forward();
                                 break;


                        case 2 : arenaUturn();forward(); state++; break;

                        case 3 : arenaStraight(); forward(); state++; break;

                        case 4 :


                                 line_follow_mm(60);
                                 stop();
                                // findLine();
                                //    stop();

                                 switch(checkForCube())
                                 {
                                     case 1 : buzzerBeep_1();
                                              if(row == 1)
                                                arenaRight_2();
                                              else
                                                arenaLeft_2();

                                              state = 9; break;

                                     //case 2 : buzzerBeep(); arenaLeft(); state = 9; break;

                                     case 2 : state = 5; break;

                                     case 0 : if(row == 1)
                                                arenaRight_2();
                                              else
                                                arenaLeft_2();

                                              state = 9; break;

                                 }

                                 forward();
                                 break;

                        case 5 : //arenaStraight();
                                 line_follow_mm(76);
                                 stop();
                              //   findLine();
                              //      stop();

/*
                                 switch(checkForCube())
                                 {
                                     case 1 : buzzerBeep();
                                              arenaUturn();
                                              state = 8; break;

                                     case 2 : arenaUturn();
                                              state = 8;
                                              break;

                                     case 0 : arenaUturn();
                                              state = 8;
                                              break;
                                 }
*/
                                 buzzerBeep_1();
                                 arenaUturn();
                                 state = 8;
                                 forward();
                                 break;

                        case 6 : arenaUturn(); forward(); state++; break;

                        case 7 : arenaStraight(); forward();state++; break;

                        case 8 : if(row == 1)
                                 {
                                     arenaLeft(); forward(); state++;
                                 }
                                 else
                                 {
                                     arenaRight(); forward();  state++;
                                 }
                                 break;

                }


                if(state == 9)
                {
                    state = 0;

                    row--;
                    velocity(forwardLeftSpeed, forwardRightSpeed);
                    forward();


                    if(row == 1)
                        errorFlag = 1;

                    else
                        errorFlag = 0;

/*

                    if(row == 0)
                    {
                        stop();
                        lcd_home();
                        lcd_string("Row = 1");
                        while(1);
                    }
*/
                }

            }
        }

        else
            findLine();

        lastError = error;

        //This flag is set in the Interrupt service routine associated by the right optical encoder when the bot has skipped a node.
        if(nodeMissedFlag == 1)
        {
            stop();
            _delay_ms(100);

            lineFollowBack();
            nodeMissedFlag = 0;
            nodeMissTracker = 0;
        }
    }


    /********************D1 ends here*************************/

    lcd_print(1, 1, row, 1);

    row = wallFollow();

    stop();
    buzzer_2();

    /***************************D2 Initialization****************/

    ungrip();
    _delay_ms(500);

    lcd_init();
    lcd_print(1, 1, row, 1);


    error = 0;
    lastError = 0;
    d2Completed = 0;
    errorFlag = 0;

    nodeMissedFlag = 0;
    nodeMissTracker = 0;

    if(row < 6)
    {
        arenaLeft();
        line_follow_mm(20);
        skipNodes(6 - row);
        arenaRight();
        state = 1;
        row = 6;
    }

    else
    {
        line_follow_mm(70);
        stop();
    }

    switch(checkForCube())
    {
        case 1 : buzzerBeep_2(); arenaRight_2(); state = 0; row = 5; a[row - 1] = 1; forward(); break;
        case 2 : state = 1; forward(); break;
        case 0 : state = 1; forward(); break;
    }

    while(!d2Completed)
    {
        error = getError();

        if(error == 0)
        {
            leftSpeed = forwardLeftSpeed;
            rightSpeed = forwardRightSpeed;
        }

        else if(error == 1)
        {
            leftSpeed = forwardLeftSpeed + 20;
            rightSpeed = forwardRightSpeed - 20;
        }

        else if(error == -1)
        {
            leftSpeed = forwardLeftSpeed - 20;
            rightSpeed = forwardRightSpeed + 20;
        }

        else if(error == 100)
        {
            if(error != lastError)
            {
                nodeMissTracker = 0;

                switch(state)
                {
                    case 0 : arenaLeft();
                             switch(checkForCube())
                             {
                                 case 1 :

                                         //
                                          buzzerBeep_2(); state = 5;
                                          if(row == 1)
                                            arenaLeft_2();
                                          else
                                            arenaRight_2();
                                          /*if(armTurned == 1)
                                          {
                                              grip(); _delay_ms(1000); faceFront(); _delay_ms(1000); ungrip();_delay_ms(1000);
                                              armTurned = 0;
                                          }*/
                                          forward();
                                          a[row - 1] = 1;
                                          break;

                                 case 2 : state = 1; forward(); break;
                                 case 0 : state = 1; forward(); break;
                             }
                             break;

                    case 1 : line_follow_mm(75);
                             stop();
                             switch(checkForCube())
                             {
                                 case 1 :
                                          buzzerBeep_2(); state = 4; arenaUturn();
                                          stop(); grip_2(); _delay_ms(500);
                                          a[row - 1] = 2;
                                          forward();
                                          break;

                                 case 2 : state = 2; forward(); break;
                                 case 0 : if(row == 1)
                                          {
                                              errorFlag = 1;
                                              buzzer_2();

                                              line_follow_mm(20);

                                              skipNodes(2);
                                              buzzer_3();
                                              stop();
                                              while(1);
                                          }

                                          state = 4; arenaUturn();
                                          stop(); grip_2(); _delay_ms(1000);
                                          forward(); break;
                             }
                             break;

                    case 2 : line_follow_mm(75); stop();
                             buzzerBeep_2();
                             a[row - 1] = 3;
                             arenaUturn(); state = 3;
                             stop(); grip_2(); _delay_ms(500);

                             forward();
                             break;

                    case 3 : state = 4;
                             break;

                    case 4 :
                             if(row == 1)
                                arenaRight_3();

                             else
                                arenaLeft_3();

                             ungrip(); _delay_ms(500); state = 5;
                             forward(); break;
                }

                if(state == 5)
                {
                    state = 0;
                    row--;
                }

                if(row == 1)
                {
                    errorFlag = 1;
                }

                else
                    errorFlag = 0;

                if(row == 0)
                {
                    buzzer_2();
                    reachEnd();
                    d2Completed = 1;
                }
            }
        }

        else
            findLine();

        lastError = error;

        if(nodeMissedFlag == 1)
        {
            stop();
            _delay_ms(100);

            lineFollowBack();
            nodeMissedFlag = 0;
            nodeMissTracker = 0;
        }

    }

    while(1);

    return 0;

}


void checkCubeBehindLeft(void)
{
    int value_up= ADC_Conversion(1)
      {
          if (value_up > 80 )
          {
              cubeBehindLeft=1;



          }
      }


}

void checkCubeBehindRight(void)
{
    int value_up= ADC_Conversion(1)
      {
          if (value_up > 80 )
          {
              cubeBehindRight=1;



          }
      }


}
void line_follow_mm(int distance)
{
    int reqCount = distance/5.338;
    int error = 0;

    ShaftCountLeft = 0;
    forward();

    while(ShaftCountLeft < reqCount || ShaftCountRight < reqCount)
    {
        error = getError();

/*
        if(error > -100 && error < 100)
        {
            leftSpeed = forwardLeftSpeed - error*kp;
            rightSpeed = forwardRightSpeed + error*kp;

            velocity(leftSpeed, rightSpeed);
        }

        else if(error == -100)
            findLine();

*/
        if(error == 0)
        {
            leftSpeed = forwardLeftSpeed;
            rightSpeed = forwardRightSpeed;
        }

        else if(error == 1)
        {
            leftSpeed = forwardLeftSpeed + 20;
            rightSpeed = forwardRightSpeed - 20;
        }

        else if(error == -1)
        {
            leftSpeed = forwardLeftSpeed - 20;
            rightSpeed = forwardRightSpeed + 20;
        }

        else if(error == 100)
            velocity(forwardLeftSpeed, forwardRightSpeed);

        else
            findLine();
    }
}

void arenaLeft(void)
{
/*
    stop();
    _delay_ms(1000);
*/
    velocity(forwardLeftSpeed, forwardRightSpeed);

    if(errorFlag == 1)
        forward_mm(60);
    else
        line_follow_mm(60);

    stop();

    nodeMissEnable = 0;

    _delay_ms(100);

/*
                soft_left();
                _delay_ms(1000);
*/
    velocity(forwardLeftSpeed, forwardRightSpeed);
    left();
    _delay_ms(1000);

    centre = ADC_Conversion(2);

    while(centre < x)
    {
        centre = ADC_Conversion(2);
    }

    stop();

    _delay_ms(100);

    nodeMissEnable = 1;
}

void arenaLeft_2()
{
    nodeMissEnable = 0;

    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);
    left();
    _delay_ms(1000);

    centre = ADC_Conversion(2);

    while(centre < x)
    {
        centre = ADC_Conversion(2);
    }

    stop();
    _delay_ms(100);

    nodeMissEnable = 1;
}

void arenaLeft_3()
{
    nodeMissEnable = 0;

    velocity(forwardLeftSpeed + 30, forwardRightSpeed + 30);
    soft_left();
    _delay_ms(1000);

    centre = ADC_Conversion(2);

    while(centre < x)
    {
        centre = ADC_Conversion(2);
    }

    stop();
    _delay_ms(100);

    nodeMissEnable = 1;
}

void arenaRight(void)
{
    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);

    if(errorFlag == 1)
        forward_mm(60);
    else
        line_follow_mm(60);

    stop();

    nodeMissEnable = 0;
    _delay_ms(100);

    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);
    right();
    _delay_ms(1000);

    centre = ADC_Conversion(2);

    while(centre < x)
    {
        centre = ADC_Conversion(2);
    }

    stop();
    _delay_ms(100);

    nodeMissEnable = 1;
}

void arenaRight_2(void)
{
    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);

    //line_follow_mm(60);
    //stop();

    nodeMissEnable = 0;
    _delay_ms(100);

    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);
    right();
    _delay_ms(1000);

    centre = ADC_Conversion(2);

    while(centre < x)
    {
        centre = ADC_Conversion(2);
    }

    stop();
    _delay_ms(100);

    nodeMissEnable = 1;
}

void arenaRight_3(void)
{
    velocity(forwardLeftSpeed, forwardRightSpeed);

    //line_follow_mm(60);
    //stop();

    nodeMissEnable = 0;
    _delay_ms(100);

    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);
    soft_right();

    _delay_ms(1000);

    centre = ADC_Conversion(2);

    while(centre < x)
    {
        centre = ADC_Conversion(2);
    }

    stop();
    _delay_ms(100);

    nodeMissEnable = 1;
}

void arenaRight_4(void)
{
    velocity(forwardLeftSpeed, forwardRightSpeed);

    //line_follow_mm(60);
    //stop();

    nodeMissEnable = 0;
    _delay_ms(100);

    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);
        left();

    _delay_ms(4000);

    centre = ADC_Conversion(2);

    while(centre < x)
    {
        centre = ADC_Conversion(2);
    }

    stop();
    _delay_ms(100);

    nodeMissEnable = 1;
}

void arenaUturn(void)
{
    nodeMissEnable = 0;
    stop();
    _delay_ms(100);

    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);
    right();
    _delay_ms(2200);

    centre = ADC_Conversion(2);

    while(centre < x)
    {
        centre = ADC_Conversion(2);
    }

    stop();
    _delay_ms(100);

    nodeMissTracker = 25;

    nodeMissEnable = 1;
}

void arenaStraight(void)
{
    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);
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

/*
    if(sensorLeft > 13 && sensorRight > 13)
        error = 100;

    else if(centre > 40)
        error = 0;

    else if(sensorLeft > 80)
        error = 4;

    else if(sensorLeft > 50)
        error = 3;

    else if(sensorLeft > 30)
        error = 2;

    else if(sensorLeft > 9)
        error = 1;

    else if(sensorRight > 80)
        error = -4;

    else if(sensorRight > 60)
        error = -3;

    else if(sensorRight > 40)
        error = -2;

    else if(sensorRight > 11)
        error = -1;

    else
        error = -100;

*/

    if(errorFlag == 1)
    {
        if((sensorLeft > 13 && centre > 30) || (sensorRight > 13 && centre > 30))
            error = 100;
    }


    else if((sensorLeft > 11 && sensorRight > 11) && errorFlag != 1)
        error = 100;

    else if(centre > 15)
        error = 0;

    else if(sensorRight > 15)
        error = 1;

    else if(sensorLeft > 15)
        error = -1;

    else
        error = -100;

    return error;
}

int checkForCube(void)
{
    nodeMissEnable = 0;

    int value = ADC_Conversion(11);
    int max = 0;

    if(value > 100)
        return 1;

    else if(value > 40)
        return 2;



    else
    {
        velocity(forwardLeftSpeed, forwardRightSpeed);

        sensorLeft = ADC_Conversion(3);
        right();
        while(sensorLeft < 20)
        {
            value = ADC_Conversion(11);
            if(value > max)
                max = value;
            sensorLeft = ADC_Conversion(3);
        }

        stop();
        _delay_ms(100);

        sensorRight = ADC_Conversion(1);
        left();
        while(sensorRight < 20)
        {
            value = ADC_Conversion(11);
            if(value > max)
                max = value;
            sensorRight = ADC_Conversion(1);
        }

        stop();
        _delay_ms(100);

        centre = ADC_Conversion(2);
        right();
        while(centre < 15)
        {
            value = ADC_Conversion(11);
            if(value > max)
                max = value;
            centre = ADC_Conversion(2);
        }

        stop();
        _delay_ms(100);

        if(max > 140)
        {
            nodeMissEnable = 1;
            return 1;
        }

        if(max > 50)
        {
            nodeMissEnable = 1;
            return 2;
        }

        nodeMissEnable = 1;
        return 0;

    }
}

void buzzerBeep_2()
{
    //This function is currently under construction. The buzzer seems to be faulty. So, the return statement is put in the first
    //line. This statement is to be removed when the buzzer is hardware rectified.
    PORTJ = 0xFF;
    _delay_ms(1000);
    PORTJ = 0x00;


/*
    if(!isAlignedWrong())
        return;
*/
   // code for turning on the  white line sensors to be written


   PORTL &=~(1 << 7);



    ungrip_2();
    _delay_ms(1000);

    if(isAlignedCorrect_2())
    {
        ungrip();
        _delay_ms(500);
        PORTL |= (1 << 7);

        buzzer_1();
        return;
    }

    grip();
    _delay_ms(1000);
    faceBack();
    _delay_ms(1000);
    ungrip();
    _delay_ms(1000);
    faceFront();
    _delay_ms(1000);

    buzzer_1();

    PORTL |= (1 << 7);
    return;

}

void findLine(void)
{
    nodeMissEnable = 0;

    int tempLeftCount = ShaftCountLeft;
    int tempRightCount = ShaftCountRight;

    ShaftCountRight = 0;
    ShaftCountLeft = 0;

    velocity(forwardLeftSpeed - 20, forwardRightSpeed - 20);

    sensorRight = ADC_Conversion(1);

    left();
    while((sensorRight < 20) && (ShaftCountRight < 8))
    {
        centre = ADC_Conversion(2);
        if(centre > x)
        {

            nodeMissEnable = 1;
            ShaftCountLeft = tempLeftCount;
            ShaftCountRight = tempRightCount;
            forward();
            return;
        }
        sensorRight = ADC_Conversion(1);
    }

    stop();
    _delay_ms(100);

    sensorLeft = ADC_Conversion(3);
    right();

    ShaftCountLeft = 0;
    ShaftCountRight = 0;

    while((sensorLeft < 20) && (ShaftCountLeft < 16))
    {
        centre = ADC_Conversion(2);
        if(centre > x)
        {
            stop();
            nodeMissEnable = 1;
            ShaftCountLeft = tempLeftCount;
            ShaftCountRight = tempRightCount;
            forward();
            return;
        }
        sensorLeft = ADC_Conversion(3);
    }
/*
    stop();
    lcd_home();
    lcd_string("Leaving the program now!");
    while(1);

*/

}

void lineFollowBack(void)
{
    nodeMissEnable = 0;

/*
    centre = ADC_Conversion(2);
    sensorLeft = ADC_Conversion(3);
    sensorRight = ADC_Conversion(1);

    int error = 0, lastError = 0;

    velocity(forwardLeftSpeed, forwardRightSpeed);
    back();

    while(!(sensorLeft > 20 && sensorRight > 20))
    {
        error = getError();
        if(error > -100 && error < 100)
        {
            leftSpeed = forwardLeftSpeed - error*kp;
            rightSpeed = forwardRightSpeed + error*kp;

            if(leftSpeed > 255)
                leftSpeed = 255;

            if(rightSpeed > 255)
                rightSpeed = 255;

            velocity(leftSpeed, rightSpeed);
        }

       // else if(error == -100)
       //     findLine();

        lastError = error;
    }

    stop();
    _delay_ms(100);

    velocity(forwardLeftSpeed, forwardRightSpeed);
*/
    ShaftCountLeft = 0;

    back();
    sensorLeft = ADC_Conversion(3);
    sensorRight = ADC_Conversion(1);

    while(!(sensorLeft > 20 && sensorRight > 20) && ShaftCountLeft < 40)
    {
         lcd_print(2, 1, sensorLeft, 3);

        lcd_print(2, 9, sensorRight, 3);
        sensorLeft = ADC_Conversion(3);
        sensorRight = ADC_Conversion(1);
    }

    stop();
    _delay_ms(100);

    forward();
    nodeMissEnable = 1;

}

void skipNodes(int n)
{
    nodeMissTracker = 0;

    int temp = 0;

    int error = 0, lastError = 0;


    velocity(forwardLeftSpeed, forwardRightSpeed);
    forward();

    error = getError();
/*
    while(error != 100)
    {
        if(error == 0)
        {
            velocity(forwardLeftSpeed, forwardRightSpeed);
        }
        else
            findLine();

        error = getError();
    }

    lastError = error;
    line_follow_mm(15);
*/
    while(temp < n)
    {
            error = getError();
            if(error == 0)
            {
                leftSpeed = forwardLeftSpeed;
                rightSpeed = forwardRightSpeed;
            }
/*
            else if(error == 1)
            {
                leftSpeed = forwardLeftSpeed + 30;
                rightSpeed = forwardRightSpeed - 30;
            }

            else if(error == -1)
            {
                leftSpeed = forwardLeftSpeed - 30;
                rightSpeed = forwardRightSpeed + 30;
            }
*/
        //Error is 100 when a node is detected.
            else if (error == 100)
            {
                if(lastError != error)
                {
                    nodeMissTracker = 0;
                    temp++;
                    leftSpeed = forwardLeftSpeed;
                    rightSpeed = forwardRightSpeed;
                }

                else
                    velocity(forwardLeftSpeed, forwardRightSpeed);
            }

            else
                findLine();

            velocity(leftSpeed, rightSpeed);

            if(nodeMissedFlag == 1)
            {
                stop();
                _delay_ms(100);

                lineFollowBack();
                nodeMissedFlag = 0;
                nodeMissTracker = 0;
            }


            lastError = error;
    }

    stop();
    return;
}

int isAlignedCorrect_2()
{
    int middle;
    //out = ADC_Conversion(14);
    middle = ADC_Conversion(15);
    //in = ADC_Conversion(12);

    if(middle > 100)
        return 0;

    else
        return 1;
}


void reachEnd()
{
        int i = 1;

        while(!(a[i] == 3 || a[i] == 0))
            i++;

        forward();

        skipNodes(i);

        stop();
        _delay_ms(1000);

        arenaRight();

        if(a[i] == 0)
        {
            skipNodes(3);
            errorFlag = 1;

            arenaRight();
            stop();
            _delay_ms(1000);

            skipNodes(i);
            stop();
            _delay_ms(1000);

            return;

            //Reach i, 1 directly
        }

        else
        {
            skipNodes(2);
            stop();
            _delay_ms(1000);
            arenaRight();
            skipNodes(1);
            arenaLeft();
            skipNodes(1);
            errorFlag = 1;

            arenaRight();
            skipNodes(i-1);
            stop();

            return;
        }
}


void buzzerBeep_1()
{
    //This function is currently under construction. The buzzer seems to be faulty. So, the return statement is put in the first
    //line. This statement is to be removed when the buzzer is hardware rectified.
    PORTJ = 0xFF;
    _delay_ms(1000);
    PORTJ = 0x00;

    if(isAlignedCorrect_1())
    {
        buzzer_1();
        return;
    }


    grip();
    _delay_ms(1000);
    faceBack();
    _delay_ms(1000);
    ungrip();
    _delay_ms(1000);
    faceFront();
    _delay_ms(1000);

    buzzer_1();
    return;

}

int isAlignedCorrect_1()
{
    int reading = ADC_Conversion(6);
    int value;

        velocity(forwardLeftSpeed, forwardRightSpeed);

        sensorLeft = ADC_Conversion(3);
        right();

        while(sensorLeft < 20)
        {
            value = ADC_Conversion(6);
            if(value < reading)
                reading = value;
            sensorLeft = ADC_Conversion(3);
        }

        stop();
        _delay_ms(100);

        sensorRight = ADC_Conversion(1);
        left();
        while(sensorRight < 20)
        {
            value = ADC_Conversion(6);
            if(value < reading)
                reading = value;
            sensorRight = ADC_Conversion(1);
        }

        stop();
        _delay_ms(100);

        centre = ADC_Conversion(2);
        right();
        while(centre < x)
        {
            value = ADC_Conversion(6);
            if(value < reading)
                reading = value;
            centre = ADC_Conversion(2);
        }

        stop();
        _delay_ms(100);



    if(state < 4)
    {
        if(reading < 120)
            return 0;

        else return 1;
    }

    else
    {
        if(reading < 120)
            return 1;

        else return 0;
    }
}

int wallFollow()
{
        int sharp, sensor, distance, row_1, error;

        velocity(forwardLeftSpeed, forwardRightSpeed);

        forward_mm(270);
        stop();

        sensor = ADC_Conversion(4);

        soft_left();

         while(sensor > 130)
            sensor = ADC_Conversion(4);

        //back_mm(30);
        stop();

        faceRight();
        _delay_ms(1000);

        sharp = getSharp();

        if(sharp < 50)
        {
            row_1 = 2;
            forward_mm(150);
        }

        else
        {
                forward();

                ShaftCountLeft = 0;




                while(sharp > 60)
                {

                    sensor = ADC_Conversion(4);


                    if(sensor < 120)
                    {
                        rightSpeed = forwardLeftSpeed - k;
                        leftSpeed = forwardRightSpeed + k;

                        velocity(leftSpeed, rightSpeed);
                    }

                    else if(sensor > 145)
                    {
                        rightSpeed = forwardLeftSpeed + k;
                        leftSpeed = forwardRightSpeed - k;

                        velocity(leftSpeed, rightSpeed);
                    }

                    else
                        velocity(forwardLeftSpeed, forwardRightSpeed);

                    sharp = getSharp();

                }



            distance = ShaftCountLeft * 5.338 + 300 + 150;

            forward_mm(290);


            row_1 = (distance)/200 + 2;
        }


        //lcd_print(1, 3, row, 1);

        stop();
        velocity(forwardLeftSpeed, forwardRightSpeed);

        faceFront();
        _delay_ms(500);

        right_degrees(90);

        forward();

        error = getError();


        while(error == -100)
            error = getError();

        forward_mm(10);

        forward();

        while(error != 100)
        {
            if(error == 0)
            {
                leftSpeed = forwardLeftSpeed;
                rightSpeed = forwardRightSpeed;
            }
/*
            else if(error == 1)
            {
                leftSpeed = forwardLeftSpeed + 20;
                rightSpeed = forwardRightSpeed - 20;
            }

            else if(error == -1)
            {
                leftSpeed = forwardLeftSpeed - 20;
                rightSpeed = forwardRightSpeed + 20;
            }
*/
            else
                findLine();

            error = getError();
        }

        stop();

        return row_1;

}

int getSharp()
{
    return ADC_Conversion(9);
}

void buzzer_1()
{
    PORTL |= (1 << 6);
    _delay_ms(500);
    PORTL &= ~(1 << 6);
}

void buzzer_2()
{
    PORTL |= (1 << 6);
    _delay_ms(1000);
    PORTL &= ~(1 << 6);
}

void buzzer_3()
{
    PORTL |= (1 << 6);
    _delay_ms(6000);
    PORTL &= ~(1 << 6);
}
