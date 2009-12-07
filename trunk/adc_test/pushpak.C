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
#include "adc.h"

#include "Print.h"
#include "HardwareSerial.h"


void setup() 
{

	Serial.begin(115200); 
	Serial.println("Pushpak Quadrotor........");
	
	GPIO_OUTPUT(LED);

	adc_initialize(); //Initialize adc at the last as this funtion enable interrupts.
	
	Serial.print("Raw ADC count for 1.1V ref = ");
	Serial.println(adc_raw_ref_val);
	Serial.print("15x accumalated ADC count for 1.1V ref = ");
	Serial.println(adc_ref_val);
	
	Serial.println();
	
	//Interrupts are enabled in the 
	//sei(); //enable interrupts

}

uint32_t testCounter;

void loop() 
{
	uint8_t	i;
	
	//Wait for System Timer Tick
	//Could also use the ADC flag to lock the sampling rate to control loop rate.
	while(gADC_new_output == 0);
	gADC_new_output = 0; //clear the flag
	
//	//use this for loop to reduce the data rate at which data is sent to pc.
//	for(i=0;i<49;++i)
//	{
//		//Wait for System Timer Tick
//		//Could also use the ADC flag to lock the sampling rate to control loop rate.
//		while(gADC_new_output == 0);
//		gADC_new_output = 0; //clear the flag

//	}

	
	GPIO_TOGGLE(LED); //LED is on Port D, Pin 4

 	adc_update_samples();

	for(i=0;i<NUM_ADC_CH;++i)
	{
		Serial.print((unsigned int) gADC_output[i]);
		Serial.print(',');
	}
	Serial.println();

//End of User code section.
/********************************************************************************/	

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

