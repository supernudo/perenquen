/*
 *  Copyright Javier Baliñas Santos (2024)
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
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>

#include "main.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "leds.h"

/* Check battery is below the min cell voltage during 5s.  */
void do_battery_check(void *dummy)
{
	static uint8_t i=0;

	#define LIPO_CELL_MIN_mv 3500
	#define BATTERY_CELLS	 2
	#define BATTERY_MIN_mV 	(BATTERY_CELLS*LIPO_CELL_MIN_mv)
	if (sensor_get_battery_mv() <= BATTERY_MIN_mV)	{
		i++;
		if(i>5) 
		{
			/* Disable BT and PWMs */
			_LATB12  = 1;
			hspwm_set_pwm(MOTOR_LEFT, 0);
			hspwm_set_pwm(MOTOR_RIGHT, 0);
			while(1) {
				LED1_TOGGLE();
				LED2_TOGGLE();
				LED3_TOGGLE();
				LED4_TOGGLE();
				wait_ms(100);
			}
		}
	}
	else {
		i = 0;
	}
}
