/*
 *  Copyright Javier Baliñas Santos (2018)
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
 *  Javier Baliñas Santos <balinas@gmail.com>
 */

#include <aversive.h>
#include <encoders_dspic.h>



#include "actuator.h"
#include "main.h"
#include "robotsim.h"

void hspwm_init(void)
{
	/* Use PWM generators 1 and 2.
		 H-bridge control type: unipolar-2Q (PWM only on half-bridge at time)
	 */

	/* Set PWM Period on Primary Time Base */
	/*
		PTPER, PHASEx, SPHASEx = FOSC / (FPWM * PWM_Input_Clock_Prescaler)
		Bit resolution = log2(PTPER)

		FPWM = desired frequency
		FOSC = Oscillator outpur (120MHz for 60 MIPS)
		PWM_Input_Clock_Prescaler = value of PCLKDIV (PTCON2 reg.)

		PTPER = 120MHZ / (20kHz x 1) = 6000

		FPWM = 20kHz
		FOSC = 120MHz
		PWM_Input_Clock_Prescaler = 1:1
	*/
	PTPER = 6000;

	/* Set Phase Shift */
	PHASE1 	= 0;
	SPHASE1 = 0;
	PHASE2 	= 0;
	SPHASE2 = 0;

	/* Set Duty Cycles */
	PDC1 = 0;
	SDC1 = 0;
	PDC2 = 0;
	SDC2 = 0;

	/* Set Dead Time Values */
	DTR1 = DTR2 = DTR3 = 0;
	ALTDTR1 = ALTDTR2 = ALTDTR3 = 0;

	/* Set PWM Mode to Independent */
	IOCON1 = IOCON2 = IOCON3 = 0xCC00;

	/* Set Primary Time Base, Edge-Aligned Mode and Independent Duty Cycles */
	PWMCON1 = PWMCON2 = PWMCON3 = 0x0000;

	/* Configure Faults */
	FCLCON1 = FCLCON2 = FCLCON3 = 0x0003;

	/* 1:1 Prescaler */
	PTCON2 = 0x0000;

	/* Enable PWM Module */
	PTCON = 0x8000;
}

void hspwm_set_pwm(void* gen_num, int16_t val) {

	switch ((int)gen_num) {
		case 1:
			/* HACK: same sign for both motors */
			//if (val >= 0) {	PDC1 = val;	SDC1 = 0; }
			//else 					{ PDC1 = 0; 	SDC1 = -val; }
			if (val >= 0) {	SDC1 = val;	PDC1 = 0; }
			else 					{ SDC1 = 0; 	PDC1 = -val; }
			break;

		case 2:
			if (val >= 0) {	PDC2 = val;	SDC2 = 0; }
			else 					{ PDC2 = 0; 	SDC2 = -val; }
      break;

		default:
			break;
	}
}

void motor_pwm_set_and_save(void *pwm_gen_num, int32_t val)
{
#define MOTOR_RIGHT_OFFSET	0
#define MOTOR_LEFT_OFFSET   0

#define MOTOR_RIGHT_MAX		(HSPWM_DUTY_MAX-MOTOR_LEFT_OFFSET)
#define MOTOR_LEFT_MAX		(HSPWM_DUTY_MAX-MOTOR_RIGHT_OFFSET)

	if (pwm_gen_num == MOTOR_LEFT) {
		/* apply offset */
		//val = val > 0? (val + MOTOR_LEFT_OFFSET):(val - MOTOR_LEFT_OFFSET);

		/* we need to do the saturation here, before saving the value */
		if (val > MOTOR_LEFT_MAX)
			val = MOTOR_LEFT_MAX;
		if (val < -MOTOR_LEFT_MAX)
			val = -MOTOR_LEFT_MAX;

		/* save value */
		mainboard.motor_pwm_left = val;
	}
	else if (pwm_gen_num == MOTOR_RIGHT){
		/* apply offset */
		//val = val > 0? (val + MOTOR_RIGHT_OFFSET):(val - MOTOR_RIGHT_OFFSET);

		/* we need to do the saturation here, before saving the value */
		if (val > MOTOR_RIGHT_MAX)
			val = MOTOR_RIGHT_MAX;
		if (val < -MOTOR_RIGHT_MAX)
			val = -MOTOR_RIGHT_MAX;

		/* save value */
		mainboard.motor_pwm_right = val;
	}

	/* set value */
#ifdef HOST_VERSION
	robotsim_pwm(pwm_gen_num, val);
#else
	hspwm_set_pwm(pwm_gen_num, val);
#endif
}
