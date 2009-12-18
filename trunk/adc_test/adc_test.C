/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "pushpak.h"
#include "util.h"
#include "adc.h"

#include "Print.h"
#include "HardwareSerial.h"

volatile uint32_t gSystemTime = 0;

struct msg_type1{
	uint16_t	header;
	uint8_t		id; 		
	uint8_t		length;
	uint32_t 	sys_time;
	int16_t	acc_x;
	int16_t	acc_y;
	int16_t	acc_z;
	int16_t	gyro_x;
	int16_t	gyro_y;
	int16_t	gyro_z;
};

struct msg_type1 msg = {0xABCD, 1}; 

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

void loop() 
{
	uint8_t	i;
	
	
//	//use this for loop to reduce the data rate at which data is sent to pc.
	for(i=0;i<50;++i)
	{
		adc_get_new_samples();
	}

	GPIO_TOGGLE(LED); //LED is on Port D, Pin 4
	
///////////////////////////////////////////////////////////////	
	//Send data in ASCII format.
// 	for(i=0;i<NUM_ADC_CH;++i)
// 	{
// 		Serial.print((unsigned int) gADC_output[i]);
// 		Serial.print(',');
// 	}
// 	Serial.println();
///////////////////////////////////////////////////////////////	
	
	//Send data in binary format.
	msg.length = sizeof(msg_type1) - 4;
	msg.sys_time = adc_get_sample_cnt();
	msg.acc_x = gADC_output[0];
	msg.acc_y = gADC_output[1];
	msg.acc_z = gADC_output[2];
	msg.gyro_x = gADC_output[3];
	msg.gyro_y = gADC_output[4];
	msg.gyro_z = gADC_output[5];
	
	Serial.write((uint8_t*)&msg, sizeof(msg_type1));
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

