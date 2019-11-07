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

struct genboard gen;
struct mainboard mainboard;

/***************************************************************/

#ifndef HOST_VERSION
void do_led_blink(void *dummy)
{
	static uint8_t i=0;

	/* simple blink */
	//LED1_TOGGLE();
	//LED2_TOGGLE();
	//LED3_TOGGLE();
	LED4_TOGGLE();

	/* Battery check */
	sensor_adc_do_read(S_ADC_BATTERY);
	if (3*sensor_adc_get_value_mv(S_ADC_BATTERY) <= 7000)	{
		i++;
		if(i>5) {
			/* Disable BT and PWMs */
			_LATB12  = 1;
			hspwm_set_pwm(MOTOR_LEFT, 0);
			hspwm_set_pwm(MOTOR_RIGHT, 0);
			while(1) {
				LED1_TOGGLE();
				//LED2_TOGGLE();
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
	_QEA1R 	= 70;	/* QEA1 <- RP70(RD6) <- R_ENC_CHA */
	_TRISD6 = 1;
	_QEB1R 	= 71;	/* QEB1 <- RP71(RD7) <- R_ENC_CHB */
	_TRISD7	= 1;

	/* XXX: encoder channels swaped for equal inc/dec sign with right encoder */
	_QEA2R 	= 97;	/* QEA2 <- RP96(RF0) <- L_ENC_CHA */
	_TRISF0 = 1;
	_QEB2R 	= 96;	/* QEB2 <- RP97(RF1)  <- L_ENC_CHB */
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
	_ANSB1 = 1;	/* AN1/RB1 FRONT-L-RX */
	_ANSB5 = 1;		/* AN5/RB5 FRONT-R-RX */
	_ANSB3 = 1;		/* AN3/RB3 DIAG-L-RX */
	_ANSB4 = 1;		/* AN4/RB4 DIAG-R-RX */
	_ANSB2 = 1;		/* AN2/RB2 FLASH-RX */

	/* battery */
	_ANSB0 = 1;		/* AN0/RB0 VBAT-SENS */

	/* gyro */
	_ANSE5 = 1;		/* AN29/RE5 GYRO-VREF */
	_ANSE6 = 1;		/* AN30/RE6 GYRO-OUTZ */

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

void hc05_programming(void) {


	/* enable interrupts */
	sei();

	printf("AT\n\r");
	wait_ms(100);
	printf("AT\n\r");
	wait_ms(100);

	printf("AT+ORGL\n\r");
	wait_ms(1000);
	printf("AT\n\r");
	wait_ms(100);

	//printf("AT+PSWD=goma\n\r");
	//wait_ms(100);

	//printf("AT+UART=115200,0,0\n\r");
	//printf("AT+UART=230400,0,0\n\r");
	//printf("AT+UART=460800,0,0\n\r");
	printf("AT+UART=921600,0,0\n\r");
	//printf("AT+UART=1382400,0,0\n\r");
	wait_ms(100);

    /* LEDS */
	LED1_ON();
	//LED2_OFF();
	LED3_ON();
	LED4_ON();

	while(1);
}

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
    mainboard.flags = DO_ENCODERS | DO_RS | DO_POS | DO_POWER;// | DO_CS; //| DO_BD;

#ifndef HOST_VERSION
	/* UART */
	uart_init();
	//hc05_programming();
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

	/* MOTORS */
	hspwm_init();
	hspwm_set_pwm(MOTOR_LEFT, 0);
	hspwm_set_pwm(MOTOR_RIGHT, 0);

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
	maindspic_cs_init();

	/* sensors, will also init hardware adc */
	sensor_init();



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

	/* say hello */
	printf("\r\n\r\n");
	printf("Hi there!! I'm Perenquen Robot :) \r\n");
	printf("Battery voltage: %d mV\n\r", 3*sensor_adc_get_value_mv(S_ADC_BATTERY));
	printf("\r\n");
	printf("M=%d\n\r", (int)M);

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

	/* init telemetry */
	tm_data_init();

	//while(1){
	//	LED1_ON();
	//	tm_data_send();
	//	LED1_OFF();
	//	tm_data_send();
	//}

	/* process commands, never returns */
	cmdline_interact();

	return 0;
}
