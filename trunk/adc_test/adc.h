/*
  This library is free software; you can redistribute it and/or modify it under the
  terms of the GNU Lesser General Public License as published by the Free Software
  Foundation; either version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT ANY
  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
  PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.
*/

// Copyright (c) 2009 Brijesh Sirpatil

#ifndef ADC_h
#define ADC_h

#include <inttypes.h>

#ifdef __cplusplus
extern "C"{
#endif

#define NUM_ADC_CH 				8	//! \def Number of ADC channels to sample

extern volatile uint16_t gADC_output[NUM_ADC_CH];	//! \def Copy of the final accumlated output result which is accessed by user programs.
extern volatile uint16_t gADC_curr[NUM_ADC_CH];		//! \def current ADC sample without any accumlation or averaging
extern uint16_t adc_ref_val, adc_raw_ref_val;		//! \def ADC sample value when 1.1V reference voltage is sampled.

void adc_initialize();
uint8_t adc_is_data_ready();
void adc_get_new_samples(void);
uint32_t adc_get_sample_cnt();

#ifdef __cplusplus
} // extern "C"
#endif

#endif
