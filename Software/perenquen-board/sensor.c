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

#ifdef NOT_USED_2015
#ifndef HOST_VERSION

static adc_init(void)
{

}

#if 0

/************ ADC */
/* config init */
static void adc2_init(void)
{
	/* Use ADC1 for allow simultaneous sampling */

	/* Set default config */
	AD2CON1 = 0;
	AD2CON2 = 0;
	AD2CON3 = 0;
	AD2CON4 = 0;

	/* Internal counter ends sampling and starts conversion (auto-convert) */
	AD2CON1bits.SSRC = 111;

	/* Channel Select bits
			1x = Converts CH0, CH1, CH2 and CH3
			01 = Converts CH0 and CH1
			00 = Converts CH0
	*/
	AD2CON2bits.CHPS = 0;

	/* Increment Rate bits
			01111 = Generates interrupt after completion of every 16th sample/conversion operation
			01110 = Generates interrupt after completion of every 15th sample/conversion operation
			•
			00001 = Generates interrupt after completion of every 2nd sample/conversion operation
			00000 = Generates interrupt after completion of every sample/conversion operation
	*/
	AD2CON2bits.SMPI = 0;

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
	AD2CON3bits.ADCS = 5;

	/* Auto-Sample Time bits
			11111 = 31 TAD
			•
			00001 = 1 TAD
			00000 = 0 TAD
	*/
	AD2CON3bits.SAMC = 2;


	/* Channel 1, 2, 3 Positive Input Select for Sample A bit
			1 = CH1 positive input is AN3, CH2 positive input is AN4, CH3 positive input is AN5
			0 = CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
	*/
	AD2CHS123bits.CH123SA = 1;

	/* Channel 0 Positive Input Select for Sample A bits */
	AD2CHS0bits.CH0SA = 0;

	/* interrupt */
	//_AD1IF = 0;
	//_AD1IE = 1;

	/* ADC module is operating */
	AD2CON1bits.ADON = 1;
}

void sensor_sample_wall_sensors(void)
{

}


/* launch new adquisition */
void adc_launch(uint16_t conf)
{
	AD1CHS0 = conf;

	/* lauch conversion */
	_SAMP = 1;
}


/* adc interrupt */
static void adc_event(uint16_t result);
void __attribute__((__interrupt__, no_auto_psv)) _ADC1Interrupt(void)
{
  	_AD1IF=0;

	/* XXX suposse _DONE = 1 */
	adc_event(ADCBUF0);
}

struct adc_infos {
	uint16_t config;
	int16_t value;
	int16_t prev_val;
  int16_t (*filter)(struct adc_infos *, int16_t);
};


/* reach 90% of the value in 4 samples */
int16_t rii_light(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + (int32_t)adc->prev_val / 2;
	return adc->prev_val / 2;
}

/* reach 90% of the value in 8 samples */
int16_t rii_medium(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + ((int32_t)adc->prev_val * 3) / 4;
	return adc->prev_val / 4;
}

/* reach 90% of the value in 16 samples */
int16_t rii_strong(struct adc_infos *adc, int16_t val)
{
	adc->prev_val = val + ((int32_t)adc->prev_val * 7) / 8;
	return adc->prev_val / 8;
}

#define ADC_CONF(x) (x)

/* define which ADC to poll, see in sensor.h */
static struct adc_infos adc_infos[ADC_MAX] = {
	[ADC_LASER_1] = { .config = ADC_CONF(7), .filter = NULL },
	[ADC_LASER_2] = { .config = ADC_CONF(6), .filter = NULL },
};

/* call on adc interrupt */
static void adc_event(uint16_t result)
{
	static uint8_t i = 0;

	/* filter value if needed */
	if (adc_infos[i].filter)
		adc_infos[i].value = adc_infos[i].filter(&adc_infos[i], result);
	else
		adc_infos[i].value = result;

	i ++;
	if (i >= ADC_MAX)
		i = 0;
	else
		adc_launch(adc_infos[i].config);
}

/* called every 10 ms, see init below */
static void do_adc(void *dummy)
{
	/* launch first conversion */
	adc_launch(adc_infos[0].config);
}
#endif /* !HOST_VERSION */
#endif

/* get analog sensor value */
int16_t sensor_get_adc(uint8_t i)
{
#ifdef HOST_VERSION
	return 0;
#else
	int16_t tmp;
	uint8_t flags;

	IRQ_LOCK(flags);
	tmp = adc_infos[i].value;
	IRQ_UNLOCK(flags);
	return tmp;
#endif
}
#endif /* NOT_USED_2015 */

/************ boolean sensors */

#ifndef HOST_VERSION
struct sensor_filter {
	uint8_t filter;
	uint8_t prev;
	uint8_t thres_off;
	uint8_t thres_on;
	uint8_t cpt;
	uint8_t invert;
};

static struct sensor_filter sensor_filter[SENSOR_MAX] = {
	[S_START_SWITCH] = { 10, 0, 0, 3, 0, 0 }, /* 0 */
	[S_RESERVED0] = { 0, 0, 0, 1, 0, 0 }, /* 1 */
	[S_RESERVED1] = { 0, 0, 0, 1, 0, 0 }, /* 2 */
	[S_RESERVED2] = { 0, 0, 0, 1, 0, 0 }, /* 3 */
	[S_RESERVED3] = { 0, 0, 0, 1, 0, 0 }, /* 4 */
	[S_RESERVED4] = { 0, 0, 0, 1, 0, 0 }, /* 5 */
	[S_RESERVED5] = { 0, 0, 0, 1, 0, 0 }, /* 6 */
	[S_RESERVED6] = { 0, 0, 0, 1, 0, 0 }, /* 7 */

	[S_GP0_0] = { 10, 0, 0, 5, 0, 0 }, /* 8 */
	[S_GP0_1] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP0_2] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP0_3] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP0_4] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP0_5] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP0_6] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP0_7] = { 10, 0, 0, 5, 0, 0 }, /* 15 */

	[S_GP1_0] = { 10, 0, 0, 5, 0, 0 }, /* 16 */
	[S_GP1_1] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP1_2] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP1_3] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP1_4] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP1_5] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP1_6] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP1_7] = { 10, 0, 0, 5, 0, 0 }, /* 23 */

	[S_GP2_0] = { 10, 0, 0, 5, 0, 0 }, /* 24 */
	[S_GP2_1] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP2_2] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP2_3] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP2_4] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP2_5] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP2_6] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP2_7] = { 10, 0, 0, 5, 0, 0 }, /* 31 */

	[S_GP3_0] = { 10, 0, 0, 5, 0, 0 }, /* 32 */
	[S_GP3_1] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP3_2] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP3_3] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP3_4] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP3_5] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP3_6] = { 10, 0, 0, 5, 0, 0 }, /*  */
	[S_GP3_7] = { 10, 0, 0, 5, 0, 0 }, /* 39 */

};
#endif /* !HOST_VERSION */

/* value of filtered sensors */
static uint64_t sensor_filtered = 0;

/* sensor mapping :
 * 0:  	  PORTA 9 (START)
 * 1-7:   reserved
 * 8-15:  i2c GP0
 * 16-23: i2c GP1
 * 24-31: i2c GP2
 * 32-39: i2c GP3
 */

uint64_t sensor_get_all(void)
{
	uint64_t tmp;
	uint8_t flags;
	IRQ_LOCK(flags);
	tmp = sensor_filtered;
	IRQ_UNLOCK(flags);
	return tmp;
}

uint8_t sensor_get(uint8_t i)
{
#ifdef HOST_VERSION
	return 0;
#else
	uint64_t tmp = sensor_get_all();
	return (uint8_t)((tmp & ((uint64_t)1 << i))>>i);
#endif
}

#ifndef HOST_VERSION
/* get the physical value of pins */
static uint64_t sensor_read(void)
{
	uint64_t tmp = 0;

	tmp |= (uint64_t)((PORTA & (_BV(9))) >> 9) << 0;
	/* 1 to 7 reserved */
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio0))<< 8;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio1))<< 16;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio2))<< 24;
	tmp |= ((uint64_t)((uint16_t)gen.i2c_gpio3))<< 32;

	/* add reserved sensors here */
	return tmp;
}

/* called every 10 ms, see init below */
static void do_boolean_sensors(void *dummy)
{
	uint8_t i;
	uint8_t flags;
	uint64_t sensor = sensor_read();
	uint64_t tmp = 0;

	for (i=0; i<SENSOR_MAX; i++) {

		if(sensor_filter[i].filter == 0)
			continue;

		if (((uint64_t)1 << i) & sensor) {
			if (sensor_filter[i].cpt < sensor_filter[i].filter)
				sensor_filter[i].cpt++;
			if (sensor_filter[i].cpt >= sensor_filter[i].thres_on)
				sensor_filter[i].prev = 1;
		}
		else {
			if (sensor_filter[i].cpt > 0)
				sensor_filter[i].cpt--;
			if (sensor_filter[i].cpt <= sensor_filter[i].thres_off)
				sensor_filter[i].prev = 0;
		}

		if (sensor_filter[i].prev && !sensor_filter[i].invert) {
			tmp |= ((uint64_t)1 << i);
		}
		else if (!sensor_filter[i].prev && sensor_filter[i].invert) {
			tmp |= ((uint64_t)1 << i);
		}

	}
	IRQ_LOCK(flags);
	sensor_filtered = tmp;
	IRQ_UNLOCK(flags);
}
#endif /* !HOST_VERSION */

/* virtual obstacle */
#define DISABLE_CPT_MAX 100 	/* XXX T_SENSORS = 10ms */
static uint16_t disable = 0; 	/* used to disable obstacle detection
			   				     * during some time
								 */

/* called every 10 ms */
void sensor_obstacle_update(void)
{
	if (disable > 0) {
		disable --;
		if (disable == 0)
			DEBUG(E_USER_STRAT, "re-enable sensor");
	}
}

void sensor_obstacle_disable(void)
{
	DEBUG(E_USER_STRAT, "disable sensor");
	disable = DISABLE_CPT_MAX;
}

void sensor_obstacle_enable(void)
{
	disable = 0;
}

uint8_t sensor_obstacle_is_disabled(void)
{
	return disable;
}


/************ global sensor init */

/* called every 10 ms, see init below */
static void do_sensors(void *dummy)
{
#ifndef HOST_VERSION
//	do_adc(NULL);
	do_boolean_sensors(NULL);
#endif
	sensor_obstacle_update();
}

void sensor_init(void)
{
#ifndef HOST_VERSION
//	adc_init();
#endif
	/* CS EVENT */
	scheduler_add_periodical_event_priority(do_sensors, NULL,
						EVENT_PERIOD_SENSORS / SCHEDULER_UNIT, EVENT_PRIORITY_SENSORS);
}
