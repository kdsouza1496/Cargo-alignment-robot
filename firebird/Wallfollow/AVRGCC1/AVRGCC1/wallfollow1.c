/*
 * wallfollow.c
 *
 * Created: 2/2/2015 9:23:50 PM
 *  Author: Kevin Dsouza
 */ 

#include <avr/io.h>

#include <util/delay.h>
#include <avr/interrupt.h>
#include "Motor.h"
#include "ADC.h"
#include "lcd.h"


void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}


void port_init(void)
{
 //servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
 //servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  
}


void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}


void init_devices(void)
{
 cli(); //disable all interrupts
 port_init();
 timer1_init();
 sei(); //re-enable interrupts 
}




#define forwardLeftSpeed 180        //Calibrated PWM value for the left motor to go straight.
#define forwardRightSpeed 153       //Calibrated PWM value for the right motor to go straight.

int leftSpeed, rightSpeed;          //Speeds of left and right motors during line following.
int centre, sensorLeft, sensorRight,value_front,value_1sens,value_4sens,value_2sens;                      //Values read from the three white line sensors respectively.




int main(void)
{
	
	cli();
    set_lcd();
    set_motors();
    set_ADC();
    sei();
	
	int getValue1(void);                      //This function returns the value of the sensor mounted on the arm 
	void findagte(void);     
	void entergate(void);
	int follow_wall(void);
	int getValue2(void) ;                              
	
	int row;
	int state;
	
	
	
   centre = ADC_Conversion(2);
   sensorLeft = ADC_Conversion(3);
   sensorRight = ADC_Conversion(1);
	 
   value_front = ADC_Conversion(11); 
   value_4sens= ADC_Conversion(12);
   value_2sens= ADC_Conversion(10);
   value_1sens=ADC_Conversion(9);
/*
    lcd_print(2, 1, sensorLeft, 3);
    lcd_print(2, 5, centre, 3);
    lcd_print(2, 9, sensorRight, 3);
*/

    if(sensorLeft <50 && sensorRight <50 && centre < 50)                 //when the bot enters the arena first
	{
		
		velocity(forwardLeftSpeed, forwardRightSpeed);
        forward_mm(350);
		stop();
        _delay_ms(100);
		
		
	}
	
	   
        //getValue2();
		 value_front = ADC_Conversion(11);
		 if(value_front>150)
		 {
			 
			 centre = ADC_Conversion(2);
			  while(centre<150)
			  {
				  
				   velocity(forwardLeftSpeed, forwardRightSpeed);
			       forward();
				   value_front = ADC_Conversion(11);
				   if(value_front<150)
				   {
					   findgate();
				   }
				 
			        centre = ADC_Conversion(2);
			  }
			
			 entergate(); 
		 }
		 
		 
		 else if(value_front < 50)
		 {
			 
			 left_degrees(90);
			 stop();
			 _delay_ms(500);
			 
			  //turn the gripper
			  servo_2(90);
			 
			 while(1)
			 {
				 
				 row=follow_wall();
				 
				 if(state)
				 {
					 soft_right();
					 
					 centre = ADC_Conversion(2);
					 while(centre<150)
					 {
						 
						velocity(forwardLeftSpeed, forwardRightSpeed);
			            forward();
				   if(value_front<150)
				   {
					   findgate();
				   }
				   
				   centre = ADC_Conversion(2);
					 }
					 entergate();
					 						 
				 }			       
				 
			 }
			 
			 	
	}		
		
			return(row); 
		 }	     
			 
		 
			 
			 
		 void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}
		 	     
	int follow_wall()
	{
		 
		unsigned long int ShaftCount;
		unsigned long int Div;
		ShaftCountRight = 0;
	    ShaftCountLeft = 0;
		int row,state;
		
		
		value_front = ADC_Conversion(11);
		
		if(value_front>150)
		{ 
			                                                                        //compute row and return 
			stop();
			ShaftCount=(ShaftCountLeft+ShaftCountRight)/2;
			row= ShaftCount/Div + 2 ;
			state=1;
			return(row) ;
			
		}
		
		
		value_1sens=ADC_Conversion(9);
		if(value_1sens > 25 && value_1sens <50 )
				 {
					 
					 
				   velocity(forwardLeftSpeed, forwardRightSpeed);
			       forward();
					 
				 }		
				 
				 else if(value_1sens < 25 && value_2sens > 50 )
				 {
					                                                                                       //right
					 right_degrees(5);
					 
					 leftSpeed=0;
					 rightSpeed=30;
					 
					  // leftSpeed = forwardLeftSpeed - error*kp;
                       //rightSpeed = forwardRightSpeed + error*kp;

                      velocity(leftSpeed, rightSpeed);
				 }
				 
				 else if(value_1sens >50 && value_2sens <25 )
				 {
					 
					  left_degrees(5);
					 
					 leftSpeed=30;
					 rightSpeed=0;
					 //leftSpeed = forwardLeftSpeed - error*kp;
                       //rightSpeed = forwardRightSpeed + error*kp;

                      velocity(leftSpeed, rightSpeed);
					                                                                                                //left
					 
				 }
				 return(0);
	}
	
	
				 		
		                                                            
																	
															
															  //front proximity sensor value 
	
    
		int getValue1(void)
{
    
	centre = ADC_Conversion(2);
   sensorLeft = ADC_Conversion(3);
   sensorRight = ADC_Conversion(1);
}
       int getValue2(void)
{	 
   value_front = ADC_Conversion(11); 
   value_4sens= ADC_Conversion(12);
   value_2sens= ADC_Conversion(10);
   value_1sens=ADC_Conversion(9);
    
}
		
		
		void findgate(void)
{
    
            
    getValue2();
	while(value_front < 150 && value_4sens >150)
	{
	
	
	/*leftSpeed = forwardLeftSpeed - *kp;
    rightSpeed = forwardRightSpeed + *kp;

    velocity(leftSpeed, rightSpeed)*/
	
    right();
	if(value_2sens >150)
	{
		forward();
		return;
	}
	getValue2();
	
	}	
	
	while(value_front < 150 && value_2sens >150)
	{
		
		/*leftSpeed = forwardLeftSpeed - *kp;
    rightSpeed = forwardRightSpeed + *kp;

    velocity(leftSpeed, rightSpeed)*/
	left();
	
	if(value >150)
	{
		forward();
		return;

	getValue2();
	}	
	
    

    stop();
    _delay_ms(100);

    

  
}

void entergate(void)
{
	//follow line 
	
	
	if (centre>150 && sensorLeft >150 && sensorRight >150)
	{
		stop();
		_delay_ms(200);
		exit(); 
	}
}

    