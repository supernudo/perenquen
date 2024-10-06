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

/** Robot dimensions */
#define ROBOT_LENGTH            101.1
#define ROBOT_WIDTH             75.75
#define ROBOT_CENTER_TO_BACK    45.8
#define ROBOT_CENTER_TO_FRONT   (ROBOT_LENGTH - ROBOT_CENTER_TO_BACK) // 55.3

#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/** Robot encoders and wheels */

/** Distance between encoders weels, decrease track to decrease angle */
#define EXT_TRACK_MM      70.0
#define VIRTUAL_TRACK_MM  EXT_TRACK_MM

/* Distance conversion factor. Increase it to go further */

/*  It is a 1024 imps 		--> 4096 because we see 1/4 period
   	Wheel diameter: 25.5mm 	--> perimeter 80.11mm
   	Distance coeficient 	--> 4096/88.11 = 51,129 imps/mm 
 */
#define IMP_ENCODERS 		1024.0
#define WHEEL_DIAMETER_MM 	25.5 // XXX: Rear wheels. Maybe 25.55 or 25.6 is OK
#define WHEEL_PERIM_MM 	    (WHEEL_DIAMETER_MM * M_PI)
#define IMP_COEF 			10.0
#define DIST_IMP_MM 		(((IMP_ENCODERS*4) / WHEEL_PERIM_MM) * IMP_COEF)

/** End of robot parameters ***************************************************/

/** Scheduler events priorities and periods */

/* XXX: These are the initial events, but tthers SW modules may add more 
		events on demand, like the trajectory manager
 */

#define EVENT_PRIORITY_BATTERY 170
#define EVENT_PRIORITY_TIME    160
#define EVENT_PRIORITY_CS      150

#define EVENT_PERIOD_BATTERY 	1000000L
#define EVENT_PERIOD_TIME		TIME_PRECISION
#define EVENT_PERIOD_CS 		1000L


/* Control system block structure */
struct cs_block {
	uint8_t on;
	struct cs cs;
  	struct pid_filter pid;
	struct quadramp_filter qr;
	struct blocking_detection bd;
};

/* General data structure */
struct genboard
{
	/* Command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* Encoders */
	#define ENCODER_LEFT  ((void*)2)
	#define ENCODER_RIGHT ((void*)1)

	/* Motors */
	#define MOTOR_LEFT    ((void*)2)
	#define MOTOR_RIGHT   ((void*)1)

	/* Logging and debug */
	#define E_USER_STRAT        194
	#define E_USER_SENSOR       196
	#define E_USER_CS           197

	#define NB_LOGS 10
	uint8_t logs[NB_LOGS+1];
	uint8_t log_level;
	uint8_t debug;
};

/* Robot data structure */
struct mainboard
{
	/* Events flags */
#define DO_ENCODERS   1
#define DO_CS         2
#define DO_RS         4
#define DO_POS        8
#define DO_BD         16
#define DO_TIMER      32
#define DO_POWER      64
#define DO_TM_DATA	  128
	uint16_t flags;

	/* Control systems */
	struct cs_block angle;
	struct cs_block distance;

	/* Robot system */
	struct robot_system rs;

	/* Robot position */
	struct robot_position pos;

	/* Robot trajectories */
  	struct trajectory traj;

	/* Current speeds */
	volatile int16_t speed_a;
	volatile int16_t speed_d;

	/* Current motors PWM */
	int32_t motor_pwm_left;
	int32_t motor_pwm_right;

};

extern struct genboard gen;
extern struct mainboard mainboard;

/* Useful macro for wait a condition or a timeout */
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
