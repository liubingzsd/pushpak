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
#include <math.h>

#include "pushpak.h"
#include "util.h"
#include "adc.h"

#include "Print.h"
#include "HardwareSerial.h"
#include "Accelerometer.h"
#include "GyroSensor.h"


void sensor_calibrate()
{
	uint8_t	i;
	const uint8_t AVG_CNT = 100;

	uint32_t gyro_x_zero, gyro_y_zero, gyro_z_zero;
	uint32_t acc_x_zeroG, acc_y_zeroG, acc_z_zeroG;
	
		
	GPIO_SET(GYRO_AUTO_ZERO); //send auto zero pulse
	_delay_us(100);
	GPIO_CLEAR(GYRO_AUTO_ZERO); 
	
	_delay_ms(100); //Gyro needs 10ms to finish auto zero, but we have low pass filter so wait longer

	acc_x_zeroG = 0;
 	acc_y_zeroG = 0;
 	acc_z_zeroG = 0;
 	gyro_x_zero = 0;
 	gyro_y_zero = 0;
 	gyro_z_zero = 0;
	
	for(i=0;i<AVG_CNT;++i) //collect data for 1sec or 100 times
	{
		adc_update();
 		acc_x_zeroG += adc_get_value_mv(ACCL_X_CH);
	 	acc_y_zeroG += adc_get_value_mv(ACCL_Y_CH);
	 	acc_z_zeroG += adc_get_value_mv(ACCL_Z_CH);
	 	
	 	gyro_x_zero += gADC_output[3];
	 	gyro_y_zero += gADC_output[4];
	 	gyro_z_zero += gADC_output[5];		
	
	}	
	
	acc_x_zeroG /= AVG_CNT;
 	acc_y_zeroG /= AVG_CNT;
 	acc_z_zeroG /= AVG_CNT;
 	gyro_x_zero /= AVG_CNT;
 	gyro_y_zero /= AVG_CNT;
 	gyro_z_zero /= AVG_CNT;
	
 	 	
 	//Note for Z axis using the same value as X axis. When sensor board is flat, the Z is either +/- 1G.
	Acclmtr.set_zerog_values((uint16_t)acc_x_zeroG, (uint16_t)acc_y_zeroG, (uint16_t) acc_x_zeroG);	 	 	
 			
	Gyro.set_zero_values((uint16_t)gyro_x_zero, (uint16_t)gyro_y_zero, (uint16_t) gyro_z_zero);	 	 	
}


void setup() 
{

	Serial.begin(115200); 
	Serial.println();
	Serial.println();
	Serial.println();
	Serial.println("Pushpak Quadrotor........");
	
	GPIO_OUTPUT(LED);
	GPIO_OUTPUT(GYRO_AUTO_ZERO);

	adc_initialize(); //Initialize adc at the last as this funtion enable interrupts.
	
	sensor_calibrate();
	
	Serial.print("Raw ADC count for 1.1V ref = ");
	Serial.println(adc_raw_ref_val);
	Serial.print("Raw ADC millivolts per count = ");
	Serial.println((float) (1100.0/(float)adc_raw_ref_val));
	
	Serial.print("Accumalated 12bit ADC count for 1.1V ref = ");
	Serial.println(adc_ref_val);
	Serial.print("Accumalated 12bit ADC millivolts per count = ");
	Serial.println((float) (1100.0/(float)adc_ref_val));
		
	Serial.print("Accelerometer calibration values, X, Y, Z: " );
	Serial.print(Acclmtr.m_zerog_x);
	Serial.print(", ");
	Serial.print(Acclmtr.m_zerog_y);
	Serial.print(", ");
	Serial.print(Acclmtr.m_zerog_z);
	Serial.println();
/*	
	Serial.print("Gyro calibration values, X, Y, Z: " );
	Serial.print(Gyro.mX_zero);
	Serial.print(", ");
	Serial.print(Gyro.mY_zero);
	Serial.print(", ");
	Serial.print(Gyro.mZ_zero);
	Serial.println();
*/
	Serial.println();
	
	//Interrupts are enabled in the 
	sei(); //enable interrupts

}

void loop() 
{
	int32_t x,y,z;
	double pitch, roll, yaw;
	int32_t temp;
	static uint8_t cnt;

	cnt++;


	
 	GPIO_TOGGLE(LED); //LED is on Port D, Pin 4
	adc_update(); //blocking call


		
	x = adc_get_value_mv(ACCL_X_CH);
	y = adc_get_value_mv(ACCL_Y_CH);
	z = adc_get_value_mv(ACCL_Z_CH);

	Acclmtr.process_ADC_sample((int16_t)x,(int16_t)y,(int16_t)z);
	Gyro.process_ADC_sample(gADC_output[3], gADC_output[4], gADC_output[5]);

	x = Acclmtr.get_x();
	y = Acclmtr.get_y();
	z = Acclmtr.get_z();

  		
	temp = y*y + z*z;
	temp = sqrt(temp);
	pitch =  atan2(x, temp);
	
	temp = x*x + z*z;
	temp = sqrt(temp);
	roll =  atan2(y, temp);

	temp = x*x + y*y;
	temp = sqrt(temp);
	yaw =  atan2(z, temp);
	
	if(cnt%50 == 0)
	{
		Serial.print(adc_get_sample_time());
		Serial.print(",");
	
		Serial.print(x);
		Serial.print(",");

		Serial.print(y);
		Serial.print(",");

		Serial.print(z);
		Serial.print(",");
	
		Serial.print(pitch);
		Serial.print(",");
	
		Serial.print(roll);
		Serial.print(",");

		Serial.println(yaw);
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

