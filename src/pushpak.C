/*
  Copyright (c) 2009 Brijesh Sirpatil
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

*/
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "pushpak.h"
#include "util.h"
#include "servo_in.h"
#include "servo_out.h"
#include "adc.h"

//////////////////////////////////////////////////////////////////
// Global variables
uint8_t gSystemError = 0;
uint8_t gSystemTimerTick = 0;
uint16_t gSystemTime = 0;
//////////////////////////////////////////////////////////////////


void timer1_init()
{
	uint16_t count;

	//Timer 1 is used by or for:
	//	a) Control Loop Timing and interrupt
	//	b) Decoding RC reciever pulses

	//With a clock prescale of 8 we get
	//Minimum interrupt time period = 400ns(2.5MHz)
	//Maxiumum interrupt time period = 26.2144ms(38.14Hz)

	//Do not use loop rate greater than 500Hz as that will mess up the RC decoding logic.
	//Safe operating loop rate in terms of not conflicting with RC reciever decoding is from
	//lower than 300Hz.

	TCCR1A = 0;
	TCCR1B = 0;

	// Prescalar 8
	SET_BIT(TCCR1B, CS11); 

	//CTC (Clear Timer on Compare Match)
	CLR_BIT(TCCR1B, WGM13); 
	SET_BIT(TCCR1B, WGM12); 
	CLR_BIT(TCCR1A, WGM11); 
	CLR_BIT(TCCR1A, WGM10); 

	//Calculate count value to generate given interrupt rate.
	//8 is clock prescale used for Timer 1
	count = (F_CPU/(CONTROL_LOOP_RATE*8));
	OCR1A = count;

	TIMSK1 = 0;
	//Enable interrupt on Compare Match A
	SET_BIT(TIMSK1, OCIE1A); 

}

//Main System timer interrupt. Used for precise control loop timing.
//The main control loop should wait on gSystemTimerTick to be set.
ISR(TIMER1_COMPA_vect){

	gSystemTimerTick = 1;
	++gSystemTime;

	GPIO_TOGGLE(LED); //LED is on Port D, Pin 4
}



void setup() 
{
  
	//Initialize all the global variables 
	gSystemError = 0;
	gSystemTimerTick = 0;
	gSystemTime = 0;

//	Serial.begin(38400); 
//	Serial.println("Servo pulse decoding......");
	
	GPIO_OUTPUT(LED);
    
	initialize_servo_in();
	timer1_init();

	servo_out_init();
	GPIO_SET(LED);
	sei(); //enable interrupts
}

void loop() 
{
	//Wait for System Timer Tick
	//Could also use the ADC flag to lock the sampling rate to control loop rate.
	while(gSystemTimerTick == 0);
	gSystemTimerTick = 0; //clear the flag
	//GPIO_TOGGLE(LED); //LED is on Port D, Pin 4
	
/********************************************************************************/
	// User code to control the Quadrotor goes here in between these comment lines.
	

	
	// Use code should not 
/********************************************************************************/	

	//Status and Error checking code
		
	
	//Every 30-40ms check if new RC reciever pulse was decoded. If no new pulse was decoded then
	//possible loss of RC radio link.
	//Mask the last 2 bits and check if they are zero. Probably faster than modulo by 4 operation.
	if((gSystemTime & 0x3) == 0)
	{
		//Check if bottom 4 bits corresponding to 4 channels of RC reciever are set
		if((gServoInStatus & 0xF) != 0xF)
		{//At least one of the receivers channels is malfunctioning
			SET_BIT(gSystemError,RC_LINK_ERROR);	
		}
	}
	
	//This should be last line in the control loop code.
	//Check if the current iteration of the code has take more time than control loop time period.	
	if(gSystemTimerTick == 1)
	{
		SET_BIT(gSystemError,EXECUTION_TIME_OVERFLOW);	
	}
}


int main(void)
{
	setup();
    
	for (;;)
	{
		loop();
	}
	    
	return 0;
}

