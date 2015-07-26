/*********************************************************************************************
#AUTHOR : SYED MOHAMMED YOUSUFF HUSSAIN.
DATE    : 15/11/2014.
*********************************************************************************************/

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder

/****************************************************************
CALL THIS FUNCTIONS BEFORE USING OTHER FUNCTIONS IN THE LIBRARY
*****************************************************************/
    void set_motors(void);

/*****************************************************************
Available functions in the library, (Total : 18)
******************************************************************/
    //Uncontrolled amount of motion. Bot will continue to move in the called direction until stop() function called.
    void forward(void);
    void back(void);
    void left(void);
    void right(void);

    //Controlled linear motion of the bot. Dimensions passed are in mm.
    void forward_mm(unsigned int);
    void back_mm(unsigned int);

    //Soft turns when one of the wheels is stationary. Turn is taken about the stationary wheel.
    void soft_left(void);
    void soft_left_2(void);

    void soft_right(void);
    void soft_right_2(void);

    //Works same as turns (left and right functions) but will turn only for specified amount of degrees.
    void left_degrees(unsigned int);
    void right_degrees(unsigned int);

    //Works same as soft turns but will turn only for specified amount of degrees.
    void soft_left_degrees(unsigned int);
    void soft_left_degrees_2(unsigned int);

    void soft_right_degrees(unsigned int);
    void soft_right_degrees_2(unsigned int);

    //This function used to set the direction of motion of the bot. Called by other functions.
    void motion_set (unsigned char);

    //This function will stop the robot.
    void stop(void);



    void motor_pin_config(void);
    void left_encoder_pin_config(void);
    void right_encoder_pin_config(void);
    void left_position_encoder_interrupt_init (void);
    void right_position_encoder_interrupt_init (void);


//Using timer5 for generating the PWM for the motors.
void timer5_init()
{
	TCCR5B = 0x00;	//Stop

	//The initial contents of the timer 5 counter register.
	//Since 8 bit PWM mode is used, we set the higher register value to 255.
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with

	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor

	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor

	//Setting the control register A value to the following configuration.
	//10100001
	//Compare mode set to clear timer on compare match.
	//Waveform generation mode set to 8 bit fast PWM mode with WGM bits(WGM0 and WGM1 in TCCR5A).
	TCCR5A |= (1<<COM5A1) | (1<<COM5B1) | (1<<WGM50);

    //WGM2 bit set for fast 8 bit PWM mode. Clock Select bits set for a prescalar of 64.
    TCCR5B |= (1<<WGM52) | (1<<CS51) | (1<<CS50);

}

void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

void motion_pin_config (void)
{
 DDRA = DDRA | 0x0F; //PORTA 0-3 (lower 4 bits) are set as output. Motors are connected at these pins.
 PORTA = PORTA & 0xF0; // Initial logic values of all the motors set to 0.
 DDRL |= (1<<3) | (1<<4);  //Setting PL3 and PL4 pins as output for PWM generation
 PORTL |= (1<<3) | (1<<4);  //PL3 and PL4 pins are for velocity control using PWM. Initially set to 1. (Max speed)
}

//Configuring left position encoder pin. Connected on PORTE 4(INT 4 pin).
void left_encoder_pin_config (void)
{
	DDRE  &= ~(1<<4);  //Set the direction of the pin as input
	PORTE |= (1<<4); //Enable internal pull-up.
}

//Configuring left position encoder pin. Connected on PORTE 5(INT 5 pin).
void right_encoder_pin_config (void)
{
	DDRE  &= ~(1<<5);  //Set the direction of the pin as input
	PORTE |= (1<<5); //Enable internal pull-up.
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt

	//Set the external interrupt to trigger at the falling edge. ISC51 = 1, ISC50 = 0
	EICRB = (1<<ISC41); // INT4 is set to trigger with falling edge
	EIMSK |= (1<<INT4); // Enable Interrupt in the interrupt mask register.

	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt

	//Set the external interrupt to trigger at the falling edge. ISC41 = 1, ISC40 = 0
	EICRB = (1<<ISC51); // INT5 is set to trigger with falling edge
	EIMSK |= (1<<INT5); // Enable Interrupt in the interrupt mask register.

	sei();   // Enables the global interrupt
}

void set_motors()
{
    motion_pin_config();
    left_encoder_pin_config();
    right_encoder_pin_config();
    left_position_encoder_interrupt_init();
    right_position_encoder_interrupt_init();
    timer5_init();
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}

//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortARestore = PORTA; 			// reading the PORTA's original status
 PortARestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortARestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTA status
 PORTA = PortARestore; 			// setting the command to the port
}


void forward (void) //both wheels forward
{
    motion_set(0x06);
}

void forward_mm(unsigned int distance)
{
    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = distance / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	ShaftCountLeft = 0;
    forward();

	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt || ShaftCountLeft > ReqdShaftCountInt)
            break;
	}
	stop(); //Stop robot
}

void back (void) //both wheels backward
{
  motion_set(0x09);
}

void back_mm (unsigned int distance)
{
    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = distance / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

    //Counting any one of the shaft counts.
	ShaftCountRight = 0;
    back();

	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void left (void) //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void left_degrees(unsigned int degrees)
{
    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

    left();

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

void right (void) //Left wheel forward, Right wheel backward
{
  motion_set(0x0A);
}

void right_degrees(unsigned int degrees)
{
    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

    right();

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}


void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_left_degrees(unsigned int degrees)
{
    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) degrees * 2 / 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

    soft_left();

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_right_degrees(unsigned int degrees)
{
    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) degrees * 2 / 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

    soft_right();

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_left_degrees_2(unsigned int degrees)
{
    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) degrees * 2 / 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

    soft_left_2();

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void right_degrees_2(unsigned int degrees)
{
    float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) degrees * 2 / 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

    soft_right_2();

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

void stop (void) //hard stop
{
  motion_set(0x00);
}

