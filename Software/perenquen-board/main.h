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
 *  Javier Bali�as Santos <javier@arc-robots.org>
 */


#ifndef _MAIN_H_
#define _MAIN_H_


#include <aversive.h>
#include <aversive/error.h>

#include <clock_time.h>
#include <rdline.h>

#include <encoders_dspic.h>
#include <dac_mc.h>
#include <pwm_servo.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>
#include <trajectory_manager.h>

#include "../common/i2c_commands.h"

/* SOME USEFUL MACROS AND VALUES  *********************************************/

/* NUMBER OF ROBOTS TO TRACK */
#define TWO_OPPONENTS
#define ROBOT_2ND

/* uart 0 is for cmds and uart 1 is
 * multiplexed between beacon and slavedspic */
#define CMDLINE_UART 	0
#define MUX_UART 		1

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
#define LED1_ON() 		cbi(LATA, 4)
#define LED1_OFF() 		sbi(LATA, 4)
#define LED1_TOGGLE() 	LED_TOGGLE(LATA, 4)

#define LED2_ON() 		cbi(LATA, 8)
#define LED2_OFF() 		sbi(LATA, 8)
#define LED2_TOGGLE() 	LED_TOGGLE(LATA, 8)

#define LED3_ON() 		cbi(LATC, 2)
#define LED3_OFF() 		sbi(LATC, 2)
#define LED3_TOGGLE() 	LED_TOGGLE(LATC, 2)

#define LED4_ON() 		cbi(LATC, 8)
#define LED4_OFF() 		sbi(LATC, 8)
#define LED4_TOGGLE() 	LED_TOGGLE(LATC, 8)


/* brake motors */
#define BRAKE_ON()      do {_LATA7 = 0; _LATB11 = 0;} while(0)
#define BRAKE_OFF()     do {_LATA7 = 1; _LATB11 = 1;} while(0)

#endif /* !HOST_VERSION */

/* only 90 seconds, don't forget it :) */
#define MATCH_TIME 89


/* ROBOT PARAMETERS *************************************************/

#undef IM_SECONDARY_ROBOT

/* distance between encoders weels,
 * decrease track to decrease angle */
#define EXT_TRACK_MM      303.626213203341000 //303.8
#define VIRTUAL_TRACK_MM  EXT_TRACK_MM

/* XXX keep synchronized with maindspic/strat.c */

/* robot dimensions */
#define ROBOT_LENGTH            288.5
#define ROBOT_WIDTH             330.0
#define ROBOT_CENTER_TO_FRONT   167.0
#define ROBOT_CENTER_TO_BACK    121.5
#define ROBOT_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK

/* XXX obstacle clerance */
#define OBS_CLERANCE            (235.+10.)
#define OBS_CLERANCE_BACKWARDS  (205.+10.)
#define OBS_CLERANCE_SIDE       (117.+20.)

/* XXX keep synchronized with secondary_robot/main.h */
#define ROBOT_SEC_LENGTH      	    163.
#define ROBOT_SEC_WIDTH 	    	210.
#define ROBOT_SEC_CENTER_TO_BACK    105.0
#define ROBOT_SEC_CENTER_TO_FRONT   (ROBOT_LENGTH-ROBOT_CENTER_TO_BACK)
#define ROBOT_SEC_HALF_LENGTH_FRONT ROBOT_CENTER_TO_FRONT
#define ROBOT_SEC_HALF_LENGTH_REAR  ROBOT_CENTER_TO_BACK
#define ROBOT_SEC_CENTER_TO_ARM		50.0

#define ROBOT_SEC_OBS_CLERANCE      (149.+10.)


/* Some calculus:
 * it is a 3600 imps -> 14400 because we see 1/4 period
 * and diameter: 55mm -> perimeter 173mm
 * 14400/173 -> 832 imps/10 mm */

/* increase it to go further */
#define IMP_ENCODERS 		    3600.0
#define WHEEL_DIAMETER_MM 	    55.0
#define WHEEL_PERIM_MM 	        (WHEEL_DIAMETER_MM * M_PI)
#define IMP_COEF 			    10.0
#define DIST_IMP_MM 		    (((IMP_ENCODERS*4) / WHEEL_PERIM_MM) * IMP_COEF)

/* encoders handlers */
#define LEFT_ENCODER        ((void *)2)
#define RIGHT_ENCODER       ((void *)1)

/* motor handles */
#define LEFT_MOTOR          ((void *)&gen.dac_mc_left)
#define RIGHT_MOTOR         ((void *)&gen.dac_mc_right)

/** ERROR NUMS */
#define E_USER_STRAT        194
#define E_USER_I2C_PROTO    195
#define E_USER_SENSOR       196
#define E_USER_CS           197
#define E_USER_BEACON       198
#define E_USER_WT11         199
#define E_USER_BT_PROTO     200


/* EVENTS PRIORITIES */
#define EVENT_PRIORITY_LED 			  170
#define EVENT_PRIORITY_TIME           160
#define EVENT_PRIORITY_I2C_POLL       140
#define EVENT_PRIORITY_SENSORS        120
#define EVENT_PRIORITY_CS             100
#define EVENT_PRIORITY_STRAT          80
#define EVENT_PRIORITY_BEACON_POLL    70

/* EVENTS PERIODS */
#define EVENT_PERIOD_LED 			1000000L
#define EVENT_PERIOD_STRAT			25000L
#define EVENT_PERIOD_BEACON_PULL    10000L
#define EVENT_PERIOD_SENSORS		10000L
#define EVENT_PERIOD_I2C_POLL		8000L
#define EVENT_PERIOD_CS 			5000L

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

/* genboard */
struct genboard
{
	/* command line interface */
	struct rdline rdl;
	char prompt[RDLINE_PROMPT_SIZE];

	/* motors */
	struct dac_mc dac_mc_left;
	struct dac_mc dac_mc_right;

	/* servos */
	struct pwm_servo pwm_servo_oc1;
	struct pwm_servo pwm_servo_oc2;

	/* i2c gpios */
	uint8_t i2c_gpio0;
	uint8_t i2c_gpio1;
	uint8_t i2c_gpio2;
	uint8_t i2c_gpio3;

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
#define DO_BEACON     128
#define DO_ROBOT_2ND  256
#define DO_BT_PROTO	(DO_BEACON | DO_ROBOT_2ND)

	/* control systems */
	struct cs_block angle;
	struct cs_block distance;

	/* x,y positionning and traj*/
	struct robot_system rs;
	struct robot_position pos;
   	struct trajectory traj;

	/* robot status */
	uint8_t our_color;

	volatile int16_t speed_a;  /* current angle speed */
	volatile int16_t speed_d;  /* current dist speed */

	int32_t dac_l;  /* current left dac */
	int32_t dac_r;  /* current right dac */

	uint8_t stored_fire_color[10];
};


/* state of slavedspic, synchronized through i2c */
struct slavedspic
{
	/* infos */
	uint8_t status;

	/* systems */
	struct {
		uint8_t mode;
		uint8_t status;
		uint8_t cup_front_catched;
		uint8_t cup_rear_catched;
		uint8_t machine_popcorns_catched;
	}popcorn_system;

	struct {
		uint8_t mode;
		uint8_t status;
		uint8_t stored_stands;
	}stands_system[I2C_SIDE_ALL];

};

/* state of beaconboard, synchronized through i2c */
struct beaconboard
{
#undef  BEACON_OFFSET
#define BEACON_OFFSET_D	30
#define BEACON_OFFSET_A	0

	/* status and color */
	uint8_t status;
	uint8_t color;
	uint8_t link_id;

	/* opponent pos */
	int16_t opponent1_x;
	int16_t opponent1_y;
	int16_t opponent1_a;
	int16_t opponent1_d;

#ifdef TWO_OPPONENTS
	int16_t opponent2_x;

	int16_t opponent2_y;
	int16_t opponent2_a;
	int16_t opponent2_d;
#endif

};

// 0: Continue with task
// 1: END
//uint8_t end_bt_task_flag;
// Current bt_task code
//uint8_t current_bt_task;

struct robot_2nd
{
  	/* bt link id */
	uint8_t link_id;

	/* running command info */
	uint8_t cmd_id;					/* for ack test */
	volatile uint8_t cmd_ret; 					/* for end traj test,
												follows END_TRAJ flags rules,
												see strat_base.h */
	/* for cmd ack test */
	uint8_t cmd_args_checksum_send;	/* checksum of arguments sent */
	uint8_t cmd_args_checksum_recv;	/* checksum received */

	volatile uint8_t valid_status;

	/* strat info */
	uint8_t color;

#define BT_TASK_NONE	   		0
#define BT_TASK_PICK_CUP   		1
#define BT_TASK_CARPET         	2
#define BT_TASK_STAIRS         	3
#define BT_TASK_BRING_CUP       4
#define BT_TASK_CLAP         	5

	uint16_t done_flags;

  	/* robot position */
	int16_t x;
	int16_t y;
  	int16_t a_abs;
	int16_t a;
	int16_t d;

  	/* opponent pos */
	int16_t opponent1_x;
	int16_t opponent1_y;
	int16_t opponent1_a;
	int16_t opponent1_d;

#ifdef TWO_OPPONENTS
	int16_t opponent2_x;
	int16_t opponent2_y;
	int16_t opponent2_a;
	int16_t opponent2_d;
#endif
};

extern struct genboard gen;
extern struct mainboard mainboard;
extern struct slavedspic slavedspic;
extern struct beaconboard beaconboard;
extern struct robot_2nd robot_2nd;


///* TODO start the bootloader */
//void bootloader(void);

#ifndef HOST_VERSION
/* swap UART 2 between beacon and slavedspic */
static inline void set_uart_mux(uint8_t channel)
{
#define BEACON_CHANNEL			0
#define SLAVEDSPIC_CHANNEL		1

	uint8_t flags;

	IRQ_LOCK(flags);


	if(channel == BEACON_CHANNEL){
		_U2RXR 	= 9;	  /* U2RX <- RP9(RB9)  <- BEACON_UART_RX */
		_TRISB9 	= 1;	/* U2RX is input								*/
	  _RP25R 	= 5;	  /* U2TX -> RP25(RC9) -> BEACON_UART_TX */
		_TRISC9	= 0;	  /* U2TX is output								*/
	}
	else
	{
		_U2RXR 	= 2;	  /* U2RX <- RP2(RB2) <- SLAVE_UART_TX	*/
		_TRISB2 	= 1;	/* U2RX is input								*/
	  _RP3R 	= 5;	  /* U2TX -> RP3(RB3) -> SLAVE_UART_RX	*/
		_TRISB3	= 0;	  /* U2TX is output								*/
	}


	IRQ_UNLOCK(flags);
	Nop();
}
#endif

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
