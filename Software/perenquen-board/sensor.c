/*
 *  Copyright Droids Corporation (2009)
 *  Olivier MATZ <zer0@droids-corp.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id$
 *
 */

/*
 *  Copyright Javier Baliñas Santos (2018)
 *  Javier Baliñas Santos <balinas@gmail.com>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  sensor.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdlib.h>

#include <aversive.h>
#include <aversive/error.h>
#include <scheduler.h>

#include "main.h"
#include "sensor.h"
#include "strat.h"
#include "strat_utils.h"


/************ ADC SENSORS *****************************************************/

/* config init */
static void adc_init(void)
{
	/* Use ADC1 for allow simultaneous sampling */

	/* Set default config */
	AD1CON1 = 0;
	AD1CON2 = 0;
	AD1CON3 = 0;
	AD1CON4 = 0;

	/* Clearing the Sample bit (SAMP) ends sampling and starts conversion (Manual mode) */
	AD1CON1bits.SSRC = 0;

	/* Channel Select bits
			1x = Converts CH0, CH1, CH2 and CH3
			01 = Converts CH0 and CH1
			00 = Converts CH0
	*/
	AD1CON2bits.CHPS = 0;

	/* Increment Rate bits
			01111 = Generates interrupt after completion of every 16th sample/conversion operation
			01110 = Generates interrupt after completion of every 15th sample/conversion operation
			•
			00001 = Generates interrupt after completion of every 2nd sample/conversion operation
			00000 = Generates interrupt after completion of every sample/conversion operation
	*/
	AD1CON2bits.SMPI = 0;

	/* ADC Conversion Clock Select bits
	11111111 = TP • (ADCS<7:0> + 1) = 256 • TCY = TAD
	•
	•
	•
	00000010 = TP • (ADCS<7:0> + 1) = 3 • TCY = TAD
	00000001 = TP • (ADCS<7:0> + 1) = 2 • TCY = TAD
	00000000 = TP • (ADCS<7:0> + 1) = 1 • TCY = TAD
	*/
	/* TCY = 1/FCY = 1/60MhZ = 16,67ns
		 TADmin@10bits = 76ns
		 ADCS = 76ns/16,67ns = 5 --> TAD = 83,335ns */
	#define ADC_TAD_ns 	(83.335)
	AD1CON3bits.ADCS = 5;

	/* Auto-Sample Time bits
			11111 = 31 TAD
			•
			00001 = 1 TAD
			00000 = 0 TAD
	*/
	//AD1CON3bits.SAMC = 2;


	/* Channel 1, 2, 3 Positive Input Select for Sample A bit
			1 = CH1 positive input is AN3, CH2 positive input is AN4, CH3 positive input is AN5
			0 = CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
	*/
	AD1CHS123bits.CH123SA = 1;

	/* Channel 0 Positive Input Select for Sample A bits */
	AD1CHS0bits.CH0SA = 0;

	/* interrupt */
	//_AD1IF = 0;
	//_AD1IE = 1;

	/* ADC module is operating */
	AD1CON1bits.ADON = 1;
}

struct adc_sensor {
	uint16_t adc_channel;
	uint16_t value;
	volatile uint16_t *enable_port;
	volatile uint8_t enable_pin;
	uint16_t delay_us;
};


static void __delay_us(uint16_t delay) {
	uint16_t i;
	for (i = 0; i < delay; i++) {
		__asm__ volatile ("repeat #39");
		__asm__ volatile ("nop");
	}
}

void do_adc_sensor_read(struct adc_sensor *adcs)
{
	uint8_t flags;

	IRQ_LOCK(flags);

	/* DEBUG */
	//LED1_ON();

	/* Set ADC channel */
	AD1CHS0bits.CH0SA = adcs->adc_channel;

	/* Enable sensor */
	if (adcs->enable_port)
		*adcs->enable_port |= (1 << adcs->enable_pin);

	/* Start sampling */
	AD1CON1bits.SAMP = 1;

	/* Wait sampling/on-sensor time */
	__delay_us(adcs->delay_us);

	/* Start conversion */
	AD1CON1bits.SAMP = 0;

	/* Disable sensor */
	if (adcs->enable_port)
		*adcs->enable_port &= ~(1 << adcs->enable_pin);

	/* Wait conversion end */
	while (!AD1CON1bits.DONE);

	/* Save result */
	adcs->value = ADC1BUF0;

	/* DEBUG */
	//LED1_OFF();

	IRQ_UNLOCK(flags);
}

/* define wall sensors read, see in sensor.h */
#define WALL_SENSORS_DELAY_US 50
static struct adc_sensor adc_sensors[S_ADC_MAX] = {
	[S_ADC_FRONT_LEFT] 			= { .adc_channel = 1, .enable_port = &LATD, .enable_pin = 5, 		.delay_us = WALL_SENSORS_DELAY_US},
	[S_ADC_FRONT_LEFT_OFF] 	= { .adc_channel = 1, .enable_port = NULL, .enable_pin = NULL, 	.delay_us = 1},
	[S_ADC_FRONT_RIGHT] 		= { .adc_channel = 5, .enable_port = &LATD, .enable_pin = 2, 		.delay_us = WALL_SENSORS_DELAY_US},
	[S_ADC_FRONT_RIGHT_OFF] = { .adc_channel = 5, .enable_port = NULL, .enable_pin = NULL, 	.delay_us = 1},
	[S_ADC_DIAG_LEFT] 			= { .adc_channel = 3, .enable_port = &LATD, .enable_pin = 4, 		.delay_us = WALL_SENSORS_DELAY_US},
	[S_ADC_DIAG_LEFT_OFF] 	= { .adc_channel = 3, .enable_port = NULL, .enable_pin = NULL, 	.delay_us = 1},
	[S_ADC_DIAG_RIGHT] 			= { .adc_channel = 4, .enable_port = &LATD, .enable_pin = 4, 		.delay_us = WALL_SENSORS_DELAY_US},
	[S_ADC_DIAG_RIGHT_OFF] 	= { .adc_channel = 4, .enable_port = NULL, .enable_pin = NULL, 	.delay_us = 1},
	[S_ADC_BATTERY] 				= { .adc_channel = 0, .enable_port = NULL, .enable_pin = NULL, 	.delay_us = 1},
	[S_ADC_GYRO] 						= { .adc_channel = 30,.enable_port = NULL, .enable_pin = NULL, 	.delay_us = 1},
	[S_ADC_GYRO_REF] 				= { .adc_channel = 29,.enable_port = NULL, .enable_pin = NULL, 	.delay_us = 1},
	[S_ADC_FLASH] 					= { .adc_channel = 2, .enable_port = NULL, .enable_pin = NULL, 	.delay_us = 1},
};

void sensor_adc_do_read (uint8_t num) {
	do_adc_sensor_read(&adc_sensors[num]);
}

uint16_t sensor_adc_get_value(uint8_t num) {
	return adc_sensors[num].value;
}

uint16_t sensor_adc_get_value_mv(uint8_t num) {
#define K_MILIVOLTS_ADC_COUNTS	(3300.0/1024)
	return (uint16_t)(adc_sensors[num].value*K_MILIVOLTS_ADC_COUNTS);
}

void sensor_init(void)
{
#ifndef HOST_VERSION
	adc_init();
#endif
}
