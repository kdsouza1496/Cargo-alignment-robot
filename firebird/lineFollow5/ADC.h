/***************************************************************
Author : SYED MOHAMMED YOUSUFF HUSSAIN
Date : 18/11/2014
****************************************************************/

/****************************************************************
CALL THIS FUNCTIONS BEFORE USING OTHER FUNCTIONS IN THE LIBRARY
*****************************************************************/
void set_ADC(void);

/****************************************************************
Available functions in the library, (Total : 2)
****************************************************************/
unsigned char ADC_Conversion(unsigned char);

void adc_pin_config()
{
    DDRF = 0x00;    //All ADC Channels on port F set as inputs
    DDRK = 0x00;    //All ADC Channels on port K set as inputs.
    PORTF = 0x00;   //All ADC pins on port F pulled low.
    PORTK = 0x00;   //All ADC pins on port K pulled low.
}

void set_ADC()
{
    adc_pin_config();

	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;        //Disable analog comparator.
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0 ( Prescalar 64)
}

//Function For ADC Conversion
//Value of ch can be 1-15 for single ended input.
//ADC resolution 8 bit.
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;

	if(Ch>7)
        ADCSRB = 0x08;

	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit

	while((ADCSRA&0x10)==0);	//Wait for conversion to complete

	a=ADCH;
	ADCSRA = ADCSRA|0x10;       //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}
