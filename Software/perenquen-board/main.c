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

#include "main.h"
#include "strat.h"
#include "cmdline.h"
#include "sensor.h"
#include "actuator.h"
#include "cs.h"

#include "robotsim.h"
#include "strat_base.h"

struct genboard gen;
struct mainboard mainboard;

/***************************************************************/

#ifndef HOST_VERSION
void do_led_blink(void *dummy)
{
	/* simple blink */
	LED1_TOGGLE();
	//LED2_TOGGLE();
	LED3_TOGGLE();
	LED4_TOGGLE();

}

static void main_timer_interrupt(void)
{
	/* scheduler tasks */
	sei();
	scheduler_interrupt();
}

/* main timer */
void main_timer_init(void)
{
	/* use timer 1 */
	T1CON = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	TMR1 = 0x0000;
	PR1 = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	T1CONbits.TON = 1;
}

/* timer 1 interrupt */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
  _T1IF=0;
  main_timer_interrupt();
}

void io_pins_init(void)
{
	/***************************************
 	*  IO portmap and config
 	*/

	/* XXX: after reset all pins are inputs */
	/* XXX: after reset all ANALOG pins are analog
	 *		  and has disabled the read operation
	 */

	/* analog inputs */
	/* set all analog pins to digital input */
	ANSELB = 0x00;
	ANSELC = 0x00;
	ANSELD = 0x00;
	ANSELE = 0x00;
	ANSELG = 0x00;

	/* leds */
	_TRISG3 = 0; /* MAIN_LED1 */
	_LATG3	= 0;
	//_TRISF6 = 0; /* MAIN_LED2 */
	//_LATF6	= 0;
	_TRISF2 = 0; /* MAIN_LED3 */
	_LATF2	= 0;
 	_TRISF3 = 0; /* MAIN_LED4 */
	_LATF3	= 0;

	/* motors */
	_TRISE3 = 0; /* PWM2H/RE3 MOTOR-L-INB */
	_LATE3	= 0;
	_TRISE2 = 0; /* PWM2L/RE2 MOTOR-L-INA */
	_LATE2	= 0;
	_TRISE1 = 0; /* PWM1H/RE1 MOTOR-R-INB */
	_LATE1	= 0;
	_TRISE0 = 0; /* PWM1L/RE0 MOTOR-R-INA */
	_LATE0	= 0;

	/* encoders */
	/* TODO: swap one encoder channels for equal inc/dec sign */
	_QEA1R 	= 96;//70;	/* QEA1 <- RP70(RD6) <- R_ENC_CHA */
	_TRISD6 = 1;
	_QEB1R 	= 97;//71;	/* QEB1 <- RP71(RD7) <- R_ENC_CHB */
	_TRISD7	= 1;

	_QEA2R 	= 70;//96;	/* QEA2 <- RP96(RF0) <- L_ENC_CHA */
	_TRISF0 = 1;
	_QEB2R 	= 71;//97;	/* QEB2 <- RP97(RF1)  <- L_ENC_CHB */
	_TRISF1	= 1;

	/* TX wall sensors */
	_TRISD5 = 0; /* FRONT-L-TX */
	_LATD5  = 0;
	_TRISD2 = 0; /* FRONT-R-TX */
	_LATD2  = 0;
	_TRISD4 = 0; /* DIAG1-TX */
	_LATD4  = 0;
	_TRISD3 = 0; /* DIAG2-TX */
	_LATD3  = 0;

	/* RX wall sensors */
	/* TODO: AN1 FRONT-L-RX */
	/* TODO: AN5 FRONT-R-RX */
	/* TODO: AN3 DIAG-L-RX */
	/* TODO: AN4 DIAG-R-RX */
	/* TODO: AN2 FLASH-RX */

	/* battery */
	/* TODO: AN0 VBAT-SENS */

	/* gyro */
	/* TODO: AN29 GYRO-VREF */
	/* TODO: AN30 GYRO-OUTZ */

	/* user switches */
	_TRISB15 = 1; /* USER_SW1 */
	_TRISB14 = 1; /* USER_SW2 */

	/* bluetooth */
	_TRISB13 = 1;	/* BT-STATE */
	_TRISB12 = 0;	/* BT-EN */
	_LATB12  = 1;

	/* uart, U1 is for cmdline */
	_U1RXR  = 101;
	_RP100R = 0b00001;
	_TRISF4 = 0;
	_TRISF5 = 1;
}
#endif /* !HOST_VERSION */


int main(void)
{
	/* disable interrupts */
	cli();

	/* TODO: eeprom magic number */

#ifndef HOST_VERSION
	/* remapeable pins */
	io_pins_init();

	/* brake motors */
	BRAKE_ON();

	/* oscillator */
	oscillator_init();

	/* LEDS */
	LED1_OFF();
	//LED2_OFF();
	LED3_OFF();
	LED4_OFF();
#endif

	/* reset data structures */
	memset(&gen, 0, sizeof(gen));
	memset(&mainboard, 0, sizeof(mainboard));

	/* init flags */
    mainboard.flags = DO_ENCODERS | DO_RS | DO_POS | DO_POWER | DO_BD | DO_CS;

#ifndef HOST_VERSION
	/* UART */
	uart_init();
	uart_register_rx_event(CMDLINE_UART, emergency);
#endif

	/* LOGS */
	error_register_emerg(mylog);
	error_register_error(mylog);
	error_register_warning(mylog);
	error_register_notice(mylog);
	error_register_debug(mylog);

#ifndef HOST_VERSION
	/* ENCODERS */
	encoders_dspic_init();

	/* TODO: MOTORS H-BRIDGES */

	/* MAIN TIMER */
	main_timer_init();
#endif

	/* SCHEDULER */
	scheduler_init();
#ifdef HOST_VERSION
	hostsim_init();
	robotsim_init();
#endif

	/* EVENTS OR INIT MODULES THAT INCLUDE EVENTS */
#ifndef HOST_VERSION
	scheduler_add_periodical_event_priority(do_led_blink, NULL,
						EVENT_PERIOD_LED / SCHEDULER_UNIT, EVENT_PRIORITY_LED);
#endif

	/* time */
	time_init(EVENT_PRIORITY_TIME);

	/* all cs management */
	// TODO maindspic_cs_init();

	/* sensors, will also init hardware adc */
	// TODO sensor_init();

	/* strat-related event */
	// TODO scheduler_add_periodical_event_priority(strat_event, NULL,
	// 					EVENT_PERIOD_STRAT / SCHEDULER_UNIT, EVENT_PRIORITY_STRAT);

	/* log setup */
 	gen.logs[0] = E_USER_STRAT;
 	gen.log_level = 5;

	/* TODO reset strat infos */
	//strat_reset_infos();

	/* enable interrupts */
	sei();

	/* wait to init of slavedspic */
	wait_ms(2000);

	/* say hello */
	printf("\r\n");
	printf("Don't turn it on, take it a part!!\r\n");

	/* process commands, never returns */
	cmdline_interact();

	return 0;
}
