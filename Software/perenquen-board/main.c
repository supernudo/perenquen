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

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#ifndef HOST_VERSION
#include <configuration_bits_config.h>
#endif

#include <uart.h>
#include <encoders_dspic.h>
#include <scheduler.h>
#include <clock_time.h>
#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>

#include <vect_base.h>
#include <lines.h>
#include <polygon.h>

#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <parse.h>
#include <rdline.h>

#include <oscillator.h>

#include "main.h"
#include "strat.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"

#include "robotsim.h"
#include "strat_base.h"

#include "telemetry.h"
#include "hc05.h"
#include "battery.h"
#include "mcu_setup.h"
#include "leds.h"

struct genboard gen;
struct mainboard mainboard;


int main(void)
{
	/* Disable interrupts */
	cli();

#ifndef HOST_VERSION
	/* Board pins setup */
	io_setup();

	/* Brake motors */
	BRAKE_ON();

	/* Oscillator setup */
	oscillator_init();

	/* Turn off LEDs */
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	LED4_OFF();
#endif /* HOST_VERSION */

	/* Clear data structures */
	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));

	/* Init task flags */
    mainboard.flags = DO_ENCODERS | DO_RS | DO_POS | DO_POWER; // | DO_CS | DO_BD;

#ifndef HOST_VERSION
	/* UART setup */
	uart_init();
	
	#ifdef CONFIG_HC05_SETUP
	/* Bluetooth setup. Never returns */
	hc05_setup();
	#endif

	/* Emergency reset mechanism. Reset is done if "pop" string is received */
	uart_register_rx_event(CMDLINE_UART, emergency);
#endif /* HOST_VERSION */

	/* Register logging functions */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

#ifndef HOST_VERSION
	/* Encoders setup */
	encoders_dspic_init();

	/* Motors PWM setup */
	hspwm_init();
	hspwm_set_pwm(MOTOR_LEFT, 0);
	hspwm_set_pwm(MOTOR_RIGHT, 0);

	/* Main timer setup */
	timer1_setup();

#endif /* HOST_VERSION */

	/* Scheduler setup */
	scheduler_init();

#ifdef HOST_VERSION
	/* Host simulation setup */
	hostsim_init();
	robotsim_init();
#endif

	/* Time setup. It register a scheduler task */
	time_init(EVENT_PRIORITY_TIME);

	/* Robot control systems setup (distance, angle, ...) */
	maindspic_cs_init();

	/* Robot sensors setup */
	sensor_init();

	/* Battery event */
#ifndef HOST_VERSION
	scheduler_add_periodical_event_priority(do_battery_check, NULL,
		EVENT_PERIOD_BATTERY / SCHEDULER_UNIT, EVENT_PRIORITY_BATTERY);
#endif

	/* Logging setup */
 	gen.logs[0] = E_USER_STRAT;
 	gen.log_level = 5;

	/* Enable interrupts */
	sei();

	/* Say hello */
	printf("\r\n\r\n");
	printf("Hi there!! I'm Perenquen Robot :) \r\n");
	printf("\r\n");
	printf("Battery voltage: %d mV\n\r", sensor_get_battery_mv());
	printf("\r\n");

	/* Init telemetry */
	tm_data_init();

	/* Process commands, never returns */
	cmdline_interact();

	return 0;
}
