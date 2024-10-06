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
#include "mcu_setup.h"


/************ ADC SENSORS *****************************************************/

/* ADC sensor structure */
struct adc_sensor 
{
	/* The parameter enable_port determines if sensor needs to measure 
	   the value_off. Value_on is measured otherwise */

	uint16_t adc_channel;
	uint16_t value_on;
	uint16_t value_off;
	volatile uint16_t *enable_port;
	volatile uint8_t enable_pin;
};

/* ADC sensor configuration */
static struct adc_sensor adc_sensors[S_ADC_MAX] = {
	[S_ADC_FRONT_LEFT] 		= { .adc_channel = 1, .enable_port = &LATD, .enable_pin = 5, 	},
	[S_ADC_FRONT_RIGHT] 	= { .adc_channel = 5, .enable_port = &LATD, .enable_pin = 2, 	},
	[S_ADC_DIAG_LEFT] 		= { .adc_channel = 3, .enable_port = &LATD, .enable_pin = 4, 	},
	[S_ADC_DIAG_RIGHT] 		= { .adc_channel = 4, .enable_port = &LATD, .enable_pin = 4, 	},
	[S_ADC_GYRO_REF] 		= { .adc_channel = 29,.enable_port = NULL,  .enable_pin = NULL, },
	[S_ADC_GYRO] 			= { .adc_channel = 30,.enable_port = NULL,  .enable_pin = NULL, },
	[S_ADC_BATTERY] 		= { .adc_channel = 0, .enable_port = NULL,  .enable_pin = NULL, },
	[S_ADC_FLASH] 			= { .adc_channel = 2, .enable_port = NULL,  .enable_pin = NULL, },
};

/* Sensors setup. Includes ADC 1 setup */
void sensor_init(void)
{
#ifndef HOST_VERSION
	adc1_setup();
	sensor_init_adc_read_sequence();
	timer2_setup();
#endif
}

/* Lauch the first ADC reading */
void sensor_init_adc_read_sequence(void) 
{
	/* First sensor */
	struct adc_sensor *adcs = &adc_sensors[S_ADC_FRONT_LEFT];

	/* Turn on the sensor */
	if (adcs->enable_port)
		*adcs->enable_port |= (1 << adcs->enable_pin);

	/* Set ADC channel */
	AD1CHS0bits.CH0SA = adcs->adc_channel;

	/* Start ADC sampling */
	AD1CON1bits.SAMP = 1;
}

/* Sequential read of all ADC sensors */
void sensor_do_adc_read_sequence(void)
{
	#define STATE_SENSOR_ON_OFF 1
	#define STATE_SENSOR_ON		2

	static uint8_t state = STATE_SENSOR_ON_OFF;
	static uint8_t index = S_ADC_FRONT_LEFT;
	struct adc_sensor *adcs;

	/* Get current sensor */
	adcs = &adc_sensors[index];

	switch (state) 
	{
		case STATE_SENSOR_ON_OFF: 
			/* Stop sampling and start conversion (aprox. 84ns) */
			AD1CON1bits.SAMP = 0;

			/* Wait for ADC conversion end */
			while (!AD1CON1bits.DONE);
		
			/* Turn off the sensor */
			if (adcs->enable_port)
				*adcs->enable_port &= ~(1 << adcs->enable_pin);

			/* Read sensor value */
			adcs->value_on = ADC1BUF0;

			/* Start ADC sampling */
			AD1CON1bits.SAMP = 1;

			state = 2;
			break;

		case STATE_SENSOR_ON:
			/* Stop sampling and start conversion (aprox. 84ns) */
			AD1CON1bits.SAMP = 0;

			/* Wait for ADC conversion end */
			while (!AD1CON1bits.DONE);

			/* Read sensor value */
			if (adcs->enable_port)
				adcs->value_off = ADC1BUF0;
			else
				adcs->value_on = ADC1BUF0;

			/* Next sensor */
			if (index == S_ADC_BATTERY) {
				index = S_ADC_FRONT_LEFT;
			}
			else {
				index++;
			}

			/* Update current sensor */
			adcs = &adc_sensors[index];

			/* Turn on the sensor */
			if (adcs->enable_port)
				*adcs->enable_port |= (1 << adcs->enable_pin);

			/* Set ADC channel */
			AD1CHS0bits.CH0SA = adcs->adc_channel;

			/* Start ADC sampling */
			AD1CON1bits.SAMP = 1;

			if (adcs->enable_port)
				state = STATE_SENSOR_ON_OFF;
			else
				state = STATE_SENSOR_ON;

			break;

		default:
			break;
	}
}

/* Returns the ADC value with the sensor enabled */
uint16_t sensor_adc_get_value_on(uint8_t num) {
#ifndef HOST_VERSION
	return adc_sensors[num].value_on;
#else
	return 0;
#endif
}

/* Return the ACD value with the sensor disabled */
uint16_t sensor_adc_get_value_off(uint8_t num) {
#ifndef HOST_VERSION
	return adc_sensors[num].value_off;
#else
	return 0;
#endif
}

/* Return the ADC value_on in mV */
uint16_t sensor_adc_get_value_mv(uint8_t num) {
#define K_MILIVOLTS_ADC_COUNTS	(3300.0/1024)
#ifndef HOST_VERSION
	return (uint16_t)(adc_sensors[num].value_on*K_MILIVOLTS_ADC_COUNTS);
#else
	return 0;
#endif
}

/* Returns the battery voltage in mV */
uint16_t sensor_get_battery_mv(void) {
#define K_BATT_RDIV	(3)
#ifndef HOST_VERSION
	return K_BATT_RDIV * sensor_adc_get_value_mv(S_ADC_BATTERY);
#else
	return 0;
#endif
}


#if 0
static void __delay_us(uint16_t delay) {
	uint16_t i;
	for (i = 0; i < delay; i++) {
		__asm__ volatile ("repeat #39");
		__asm__ volatile ("nop");
	}
}

void do_adc_wall_sensor(struct adc_sensor *adcs)
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

//void sensor_adc_do_read (uint8_t num) {
//#ifndef HOST_VERSION
//	do_adc_sensor_read(&adc_sensors[num]);
//#endif
//}


	#if 0
	while (1) {
		uint8_t flags;

		/* Wall-sensors test */
		_LATD5  = 0;
		_LATD2  = 0;
		_LATD4  = 0;
		_LATD3  = 0;
		wait_us(900);

		IRQ_LOCK(flags);
		_LATD5  = 1;
		_LATD2  = 1;
		_LATD4  = 1;
		//_LATD3  = 1;
		wait_us(50);
		IRQ_UNLOCK(flags);
	}
	#endif

#endif


