/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* ULP Example: using ADC in deep sleep */

/* ULP assembly files are passed through C preprocessor first, so include directives
   and C macros may be used in these files
 */
#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"
#include "example_config.h"

#include "soc/sens_reg.h"

	.set adc_channel, EXAMPLE_ADC_CHANNEL

	/* Configure the number of ADC samples to average on each measurement.
	   For convenience, make it a power of 2. */
	.set adc_oversampling_factor_log, 2
	.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	/* Low threshold of ADC reading.
	   Set by the main program. */
	.global low_thr
low_thr:
	.long 0

	/* High threshold of ADC reading.
	   Set by the main program. */
	.global high_thr
high_thr:
	.long 0

	.global high
high:
	.long 0

	/* Counter of measurements done */
	.global sample_counter
sample_counter:
	.long 0

	.global previous_result
previous_result:
	.long 0

	.global last_result
last_result:
	.long 0

	.global charge_state
charge_state:
	.long 0

	/*store random reading from temperature sensor*/
	.global temperature_result
temperature_result:
	.long 0

	.global adc_reading
adc_reading:
	.long 0

	.global higher_thr
higher_thr:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	/* increment sample counter */
	move r3, sample_counter
	ld r2, r3, 0
	add r2, r2, 1
	st r2, r3, 0

	/* do measurements using ADC */
	/* r0 will be used as accumulator */
	move r0, 0
	/* initialize the loop counter */
	stage_rst

measure:
	/*
	tsens r1, 10
	move r3, temperature_result
	st r1, r3, 0
	*/

	adc r1, 1, adc_channel + 3 //saradc2
	add r0, r0, r1
	rsh r0, r0, adc_oversampling_factor_log
	move r3, adc_reading
	st r0, r3, 0

	/* measure first level */
	adc r1, 1, adc_channel + 1 //saradc2
	add r0, r0, r1
	rsh r0, r0, adc_oversampling_factor_log
	move r3, previous_result
	st r0, r3, 0

//waits for 4s
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
		wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
		wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
		wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
		wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
		wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535
	wait 65535

	/* measure next level */
	adc r1, 1, adc_channel + 1 //saradc2
	add r0, r0, r1
	
	/* divide accumulator by adc_oversampling_factor.
	   Since it is chosen as a power of two, use right shift */
	rsh r0, r0, adc_oversampling_factor_log
	/* averaged value is now in r0; store it into last_result */
	move r3, last_result
	st r0, r3, 0

	move r3, previous_result
	ld r1, r3, 0
	sub r0, r0, r1

	move r1, charge_state
	st r0, r1, 0
	
	/* compare with low_thr; power down if value < low_thr */
	move r3, low_thr
	ld r3, r3, 0
	sub r3, r0, r3
	jump hibernate, ov

	//goes to sleep if negative detected 	
	move r3, higher_thr
	ld r3, r3, 0
	sub r3, r3, r0
	jump hibernate, ov

	/* compare with high_thr; wake up if value > high_thr */
	move r3, high_thr 
	ld r3, r3, 0
	sub r3, r3, r0
	jump wake_up , ov 

	jump hibernate

	.global hibernate 
hibernate:
	/* ulp puts itself into sleep mode */
	sleep 0
	halt  

	.global wake_up 
wake_up:
	/* Check if the system can be woken up */
	READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
	and r0, r0, 1
	jump hibernate, eq 

	/* Wake up the SoC, end program */
	wake
	WRITE_RTC_FIELD(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN, 0)
	halt

	/* value within range, end the program */
