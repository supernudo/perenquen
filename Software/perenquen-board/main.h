/*
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2010)
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


#ifndef _MAIN_H_
#define _MAIN_H_


#include <aversive.h>
#include <aversive/error.h>

#include <clock_time.h>
#include <rdline.h>

#include <encoders_dspic.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>
#include <trajectory_manager.h>



/* SOME USEFUL MACROS AND VALUES  *********************************************/

/* uart 0 is for cmds and uart 1 is
 * multiplexed between beacon and slavedspic */
#define CMDLINE_UART 	0

/* generic led toggle macro */
#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		    \
			port &= ~_BV(bit);	        \
		else				            \
			port |= _BV(bit);	        \
	} while(0)

#ifdef HOST_VERSION
#define LED1_ON()
#define LED1_OFF()
#define LED1_TOGGLE()

#define LED2_ON()
#define LED2_OFF()
#define LED2_TOGGLE()

#define LED3_ON()
#define LED3_OFF()
#define LED3_TOGGLE()

#define LED4_ON()
#define LED4_OFF()
#define LED4_TOGGLE()

#define BRAKE_DDR()
#define BRAKE_ON()
#define BRAKE_OFF()

#else

/* leds manage */

/* leds */
// _TRISG3 = 0; /* MAIN_LED1 */
// _TRISF6 = 0; /* MAIN_LED2 */
// _TRISF2 = 0; /* MAIN_LED3 */
// _TRISF3 = 0; /* MAIN_LED4 */

#define LED1_ON() 		sbi(LATG, 3)
#define LED1_OFF() 		cbi(LATG, 3)
#define LED1_TOGGLE() LED_TOGGLE(LATG, 3)

//#define LED2_ON() 		sbi(LATF, 6)
//#define LED2_OFF() 		cbi(LATF, 6)
//#define LED2_TOGGLE() LED_TOGGLE(LATF, 6)

#define LED3_ON() 		sbi(LATF, 2)
#define LED3_OFF() 		cbi(LATF, 2)
#define LED3_TOGGLE() LED_TOGGLE(LATF, 2)

#define LED4_ON() 		sbi(LATF, 3)
#define LED4_OFF() 		cbi(LATF, 3)
#define LED4_TOGGLE() LED_TOGGLE(LATF, 3)


/* brake motors */
// _TRISE3 = 0; /* PWM2H/RE3 MOTOR-L-INB */
// _LATE3	= 0;
// _TRISE2 = 0; /* PWM2L/RE2 MOTOR-L-INA */
// _LATE2	= 0;
// _TRISE1 = 0; /* PWM1H/RE1 MOTOR-R-INB */
// _LATE1	= 0;
// _TRISE0 = 0; /* PWM1L/RE0 MOTOR-R-INA */
// _LATE0	= 0;

// TODO: apply brake thru PWM module
#define BRAKE_ON()      do {	hspwm_set_pwm(MOTOR_LEFT, 0); hspwm_set_pwm(MOTOR_RIGHT, 0);\
															_LATE0 = 0; _LATE1 = 0; _LATE2 = 0; _LATE3 = 0; } while(0)
#define BRAKE_OFF()     do {} while(0)

#endif /* !HOST_VERSION */


/* ROBOT PARAMETERS *************************************************/

/* distance between encoders weels,
 * decrease track to decrease angle */
#define EXT_TRACK_MM      67.0
#define VIRTUAL_TRACK_MM  EXT_TRACK_MM

/* XXX keep synchronized with maindspic/strat.c */

/* robot dimensions */
#define ROBOT_LENGTH            101.0
#define ROBOT_WIDTH             75.5
#define ROBOT_CENTER_TO_FRONT   55.5
#define ROBOT_CENTER_TO_BACK    (ROBOT_LENGTH - ROBOT_CENTER_TO_FRONT)
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* Some calculus:
 * it is a 1024 imps -> 4096 because we see 1/4 period
 * and diameter: 25.5mm -> perimeter 80.11mm
 * 4096/88.11 -> 51,129 imps/mm */

/* increase it to go further */
#define IMP_ENCODERS 		    1024.0
#define WHEEL_DIAMETER_MM 	25.5
#define WHEEL_PERIM_MM 	    (WHEEL_DIAMETER_MM * M_PI)
#define IMP_COEF 			    	10.0
#define DIST_IMP_MM 		    (((IMP_ENCODERS*4) / WHEEL_PERIM_MM) * IMP_COEF)

/** ERROR NUMS */
#define E_USER_STRAT        194
#define E_USER_SENSOR       196
#define E_USER_CS           197


/* EVENTS PRIORITIES */
#define EVENT_PRIORITY_LED 	   170
#define EVENT_PRIORITY_TIME    160
#define EVENT_PRIORITY_CS      150
#define EVENT_PRIORITY_SENSORS 120
#define EVENT_PRIORITY_STRAT   80

/* EVENTS PERIODS */
#define EVENT_PERIOD_LED 			1000000L
#define EVENT_PERIOD_STRAT		25000L
#define EVENT_PERIOD_SENSORS	10000L
#define EVENT_PERIOD_CS 			1000L

#define CS_PERIOD   ((EVENT_PERIOD_CS/SCHEDULER_UNIT)*SCHEDULER_UNIT) /* in microsecond */
#define CS_HZ       (1000000. / CS_PERIOD)

/* dynamic logs */
#define NB_LOGS 10

/* MAIN DATA STRUCTURES **************************************/

/* cs data */
struct cs_block {
	uint8_t on;
	struct cs cs;
  struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

struct tm_cs_block {
	/* total 5 x 4 bytes = 20 bytes */
	int32_t consign;
	int32_t fconsign;
	int32_t error;
	int32_t ffeedback;
	int32_t out;
};

struct tm_block {
	uint8_t header[4];						// 4 bytes
	uint32_t time_ms;							// 4 bytes
	struct tm_cs_block angle;			// 20 bytes
	struct tm_cs_block distance;	// 20 bytes
																// 48 bytes total

	// wall sensors: 4 x 2 = 8 bytes
	// gyro: 								 2 bytes
	// battery: 						 1 byte
	// encoders:		 2 x 4 = 8 bytes
	// x, y, a:			 3 x 2 = 6 bytes
	// i, j:				 2 x 1 = 2 bytes
	// slot_info:						 1 byte
	// total:							  74 bytes
};

/* genboard */
struct genboard
{
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* encoders */
	#define ENCODER_LEFT  ((void*)2)
	#define ENCODER_RIGHT ((void*)1)

	/* motors */
	#define MOTOR_LEFT    ((void*)2)
	#define MOTOR_RIGHT   ((void*)1)

	/* TODO battery */

	/* TODO wall-sensors */

	/* log */
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;
};

/* maindspic */
struct mainboard
{
	/* events flags */
	uint16_t flags;
#define DO_ENCODERS   1
#define DO_CS         2
#define DO_RS         4
#define DO_POS        8
#define DO_BD         16
#define DO_TIMER      32
#define DO_POWER      64
#define DO_TM_DATA		128

	uint8_t our_color;
	#define I2C_COLOR_YELLOW	0
	#define AREA_X 10
	#define AREA_Y 10
  #define OBS_CLERANCE 1

	/* control systems */
	struct cs_block angle;
	struct cs_block distance;

	/* x,y positionning and traj*/
	struct robot_system rs;
	struct robot_position pos;
  struct trajectory traj;

	/* TODO review */
	volatile int16_t speed_a;  /* current angle speed */
	volatile int16_t speed_d;  /* current dist speed */

	/* current motors pwm */
	int32_t motor_pwm_left;
	int32_t motor_pwm_right;

	/* telemetry data */
	struct tm_block tm;

};

extern struct genboard gen;
extern struct mainboard mainboard;


#define WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
	if (__ret)					      \
		DEBUG(E_USER_STRAT, "cond is true at line %d",\
		      __LINE__);			      \
	else						      \
		DEBUG(E_USER_STRAT, "timeout at line %d",     \
		      __LINE__);			      \
							      \
        __ret;                                                \
})
#endif
