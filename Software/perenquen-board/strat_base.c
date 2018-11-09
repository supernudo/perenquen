/*
 *  Copyright Droids Corporation, Microb Technology (2009)
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
 *  strat_base.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>


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

#include <rdline.h>
#include <parse.h>



#include "main.h"
#include "cmdline.h"
#include "strat_utils.h"
#include "strat_base.h"
#include "strat.h"
#include "sensor.h"



/* true if we want to interrupt a trajectory */
static uint8_t traj_intr=0;

/* filled when a END_OBSTACLE is returned */
struct opponent_obstacle opponent_obstacle;

/* asked speed */
static volatile int16_t strat_speed_a = SPEED_DIST_FAST;
static volatile int16_t strat_speed_d = SPEED_ANGLE_FAST;
static volatile uint16_t strat_limit_speed_a = 0; /* no limit */
static volatile uint16_t strat_limit_speed_d = 0;

static volatile uint8_t strat_limit_speed_enabled = 1;

/* opponent front/rear sensors for obstacle detection */
static volatile uint8_t strat_opp_sensors_enabled = 0;
static volatile uint8_t strat_opp_sensor_middle_enabled = 0;

/* Strings that match the end traj cause */
/* /!\ keep it sync with stat_base.h */
char *err_tab []= {
	"END_TRAJ",
	"END_BLOCKING",
	"END_NEAR",
	"END_OBSTACLE",
	"END_ERROR",
	"END_INTR",
	"END_TIMER",
	"END_RESERVED",
};

/* return string from end traj type num */
const char *get_err(uint8_t err)
{
	uint8_t i;
	if (err == 0)
		return "SUCCESS";
	for (i=0 ; i<8; i++) {
		if (err & (1 <<i))
			return err_tab[i];
	}
	return "END_UNKNOWN";
}

void strat_hardstop(void)
{
	trajectory_hardstop(&mainboard.traj);
	pid_reset(&mainboard.angle.pid);
	pid_reset(&mainboard.distance.pid);
	bd_reset(&mainboard.angle.bd);
	bd_reset(&mainboard.distance.bd);

	while ((ABS(mainboard.speed_d) > 200) ||
	       (ABS(mainboard.speed_a) > 200))

	trajectory_hardstop(&mainboard.traj);
	pid_reset(&mainboard.angle.pid);
	pid_reset(&mainboard.distance.pid);
	bd_reset(&mainboard.angle.bd);
	bd_reset(&mainboard.distance.bd);
}
/* reset position */
void strat_reset_pos(int16_t x, int16_t y, int16_t a)
{
	int16_t posx = position_get_x_s16(&mainboard.pos);
	int16_t posy = position_get_y_s16(&mainboard.pos);
	int16_t posa = position_get_a_deg_s16(&mainboard.pos);

	if (x == DO_NOT_SET_POS)
		x = posx;
	if (y == DO_NOT_SET_POS)
		y = posy;
	if (a == DO_NOT_SET_POS)
		a = posa;

	/* some issues with blocking on simulator */
#ifdef HOST_VERSION
	if (x == ROBOT_HALF_LENGTH_REAR)
		x = ROBOT_HALF_LENGTH_REAR + 10;
	if (x == AREA_X - ROBOT_HALF_LENGTH_REAR)
		x = AREA_X - ROBOT_HALF_LENGTH_REAR - 10;
	if (y == ROBOT_HALF_LENGTH_REAR)
		y = ROBOT_HALF_LENGTH_REAR + 10;
	if (y == AREA_Y - ROBOT_HALF_LENGTH_REAR)
		y = AREA_Y - ROBOT_HALF_LENGTH_REAR - 10;
	if (x == ROBOT_HALF_LENGTH_FRONT)
		x = ROBOT_HALF_LENGTH_FRONT + 10;
	if (x == AREA_X - ROBOT_HALF_LENGTH_FRONT)
		x = AREA_X - ROBOT_HALF_LENGTH_FRONT - 10;
	if (y == ROBOT_HALF_LENGTH_FRONT)
		y = ROBOT_HALF_LENGTH_FRONT + 10;
	if (y == AREA_Y - ROBOT_HALF_LENGTH_FRONT)
		y = AREA_Y - ROBOT_HALF_LENGTH_FRONT - 10;
#endif

	DEBUG(E_USER_STRAT, "reset pos (%s%s%s)",
	      x == DO_NOT_SET_POS ? "" : "x",
	      y == DO_NOT_SET_POS ? "" : "y",
	      a == DO_NOT_SET_POS ? "" : "a");
	position_set(&mainboard.pos, x, y, a);
	DEBUG(E_USER_STRAT, "pos resetted", __FUNCTION__);
}

/*
 * decrease gain on angle PID, and go forward until we reach the
 * border.
 */
uint8_t strat_calib(int16_t dist, uint8_t flags)
{
	int32_t p = pid_get_gain_P(&mainboard.angle.pid);
	int32_t i = pid_get_gain_I(&mainboard.angle.pid);
	int32_t d = pid_get_gain_D(&mainboard.angle.pid);
	uint8_t err;

	pid_set_gains(&mainboard.angle.pid, 50, 0, 1000);
	trajectory_d_rel(&mainboard.traj, dist);
	err = wait_traj_end(flags);
	pid_set_gains(&mainboard.angle.pid, p, i, d);

	return err;
}

static void strat_update_traj_speed(void)
{
	uint16_t d, a;

	d = strat_speed_d;
	if (strat_limit_speed_d && d > strat_limit_speed_d)
		d = strat_limit_speed_d;
	a = strat_speed_a;
	if (strat_limit_speed_a && a > strat_limit_speed_a)
		a = strat_limit_speed_a;

	trajectory_set_speed(&mainboard.traj, d, a);
}

void strat_set_speed(uint16_t d, uint16_t a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	strat_speed_d = d;
	strat_speed_a = a;
	strat_update_traj_speed();
	IRQ_UNLOCK(flags);
}

#ifdef TRAJECTORY_MANAGER_V3
void strat_set_acc(double d, double a)
{
	trajectory_set_acc(&mainboard.traj, d, a);
}
#endif

void strat_get_speed(uint16_t *d, uint16_t *a)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	*d = strat_speed_d;
	*a = strat_speed_a;
	IRQ_UNLOCK(flags);
}

void strat_limit_speed_enable(void)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	strat_limit_speed_enabled = 1;
	IRQ_UNLOCK(flags);
}

void strat_limit_speed_disable(void)
{
	uint8_t flags;
	IRQ_LOCK(flags);
	strat_limit_speed_enabled = 0;
	IRQ_UNLOCK(flags);
}

/* start the strat */
void strat_start(void)
{
	//uint8_t i;
    uint8_t err;

	/* dump strat configuration, set do flags, ... */
	strat_preinit();

#ifndef HOST_VERSION
	// TODO
	/* if start sw not plugged */
	//if (!sensor_get(S_START_SWITCH)) {
	//	printf_P(PSTR("NO START SWITCH, press a key or plug it\r\n"));

	//	/* while start sw not plugged */
	//	while (!sensor_get(S_START_SWITCH)) {
	//		if (!cmdline_keypressed())
	//			continue;

	//		for (i=3; i>0; i--) {
	//			printf_P(PSTR("%d\r\n"), i);
	//			time_wait_ms(1000);
	//		}
	//		break;
	//	}
	//}

	// TODO
	/* if start sw plugged */
	//if (sensor_get(S_START_SWITCH)) {
	//	printf_P(PSTR("READY!!!, unplug start switch to start\r\n"));
	//	/* while start sw plugged */
	//	while (sensor_get(S_START_SWITCH));
	//}
#endif

	printf_P(PSTR("GOO!!!\r\n"));

	/* reset infos, set traj speeds, set events ...*/
	strat_init();

	/* go to play */
	err = strat_main();

	NOTICE(E_USER_STRAT, "Finished!! returned %s", get_err(err));
	strat_exit();
}


/* interrupt the current trajectory, set a flat to TRUE */
void interrupt_traj(void)
{
	traj_intr = 1;
}

/* reset interrupt trajectory flag to FALSE*/
void interrupt_traj_reset(void)
{
	traj_intr = 0;
}

/* testing of several posible ends for a trajectory */
uint8_t test_traj_end(uint8_t why)
{
	//uint16_t cur_timer;
	point_t robot_pt;

	robot_pt.x = position_get_x_s16(&mainboard.pos);
	robot_pt.y = position_get_y_s16(&mainboard.pos);

	/* hardstop ends */
	if ((why & END_BLOCKING) && bd_get(&mainboard.angle.bd)) {
		strat_hardstop();
		return END_BLOCKING;
	}

	if ((why & END_BLOCKING) && bd_get(&mainboard.distance.bd)) {
		strat_hardstop();
		return END_BLOCKING;
	}

	//if ((why & END_OBSTACLE) && strat_obstacle()) {
	//	strat_hardstop();
	//	return END_OBSTACLE;
	//}

	/* interrupt traj by user */
	if ((why & END_INTR) && traj_intr) {
		interrupt_traj_reset();
		return END_INTR;
	}

	/* traj ends succesfully */
	if ((why & END_TRAJ) && trajectory_finished(&mainboard.traj))
		return END_TRAJ;

	/* trigger an event at 3 sec before the end of the match */
	//cur_timer = time_get_s();
	//if ((mainboard.flags & DO_TIMER) && (why & END_TIMER)) {
	//	/* end of match */
	//	if (cur_timer >= MATCH_TIME)
	//		return END_TIMER;
	//}

	/* we are near the destination point (depends on current
	 * speed) AND the robot is in the area bounding box. */
	if (why & END_NEAR) {
		int16_t d_near = 100;

    	/* XXX */
		if (mainboard.speed_d >= 2000)
			d_near = 150;

		if (trajectory_in_window(&mainboard.traj, d_near, RAD(5.0)) &&
		    is_in_boundingbox(&robot_pt))
			return END_NEAR;
	}

	return 0;
}

/* wait the ends of a trajectory (several ends are possible) */
uint8_t __wait_traj_end_debug(uint8_t why, uint16_t line)
{
	uint8_t ret = 0;
	//int16_t opp_x, opp_y, opp_d, opp_a;

	while (ret == 0){
		ret = test_traj_end(why);
	}
	if (ret == END_OBSTACLE) {
		//if (get_opponent1_xyda(&opp_x, &opp_y,
		//		      &opp_d, &opp_a) != -1)
		//	DEBUG(E_USER_STRAT, "Got %s at line %d"
		//	      " opp1 xy=(%d,%d) da=(%d,%d)", get_err(ret),
		//	      line, opp_x, opp_y, opp_d, opp_a);
    //    if (get_opponent2_xyda(&opp_x, &opp_y,
		//		      &opp_d, &opp_a) != -1)
		//	DEBUG(E_USER_STRAT, "Got %s at line %d"
		//	      " opp2 xy=(%d,%d) da=(%d,%d)", get_err(ret),
		//	      line, opp_x, opp_y, opp_d, opp_a);
	}
	else {
		DEBUG(E_USER_STRAT, "Got %s at line %d",
		      get_err(ret), line);
	}
	return ret;
}
