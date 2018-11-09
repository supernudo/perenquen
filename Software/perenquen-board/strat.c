/*
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2012)
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
 *  Javier Balinas Santos <javier@arc-robots.org> and Silvia Santano
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/queue.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#ifndef HOST_VERSION_OA_TEST
#include <clock_time.h>
#include <uart.h>



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
#include "strat.h"
#include "strat_base.h"

#include "strat_utils.h"
#include "sensor.h"
#include "actuator.h"

#endif

#define LIMIT_BBOX_Y_UP			(2000 - OBS_CLERANCE-70)
#define LIMIT_BBOX_Y_DOWN		OBS_CLERANCE+100
#define LIMIT_BBOX_X_UP			3000 - OBS_CLERANCE
#define LIMIT_BBOX_X_DOWN		OBS_CLERANCE


struct strat_infos strat_infos = {
    /* conf */
    .conf = {
        .flags = 0,
    },
};



/*************************************************************/

/*                  INIT                                     */

/*************************************************************/
void strat_set_bounding_box(void)
{

    strat_infos.area_bbox.x1 = OBS_CLERANCE;
    strat_infos.area_bbox.x2 = 3000 - OBS_CLERANCE;

    strat_infos.area_bbox.y1 = OBS_CLERANCE-5;

    strat_infos.area_bbox.y2 = 2000 - OBS_CLERANCE-70;

    polygon_set_boundingbox(strat_infos.area_bbox.x1,
                strat_infos.area_bbox.y1,
                strat_infos.area_bbox.x2,
                strat_infos.area_bbox.y2);
#ifdef HOST_VERSION_OA_TEST

    printf("boundingbox at: %d %d %d %d\n",
                strat_infos.area_bbox.x1,
                strat_infos.area_bbox.y1,
                strat_infos.area_bbox.x2,
                strat_infos.area_bbox.y2);
#endif
}

#ifndef HOST_VERSION_OA_TEST

/* display curret strat configuration */
void strat_dump_conf(void)
{
    if (!strat_infos.dump_enabled)
        return;

    printf_P(PSTR("-- conf --\r\n"));

    /* add here configuration dump */
}


/* display current information about the state of the game */
void strat_dump_infos(const char *caller)
{
    if (!strat_infos.dump_enabled)
        return;

    printf(PSTR("%s() dump strat infos:\r\n"), caller);

    /* TODO: add here print infos */
}

/* init current area state before a match. Dump update user conf
 * here */
void strat_reset_infos(void)
{
    /* bounding box */
    strat_set_bounding_box();

    /* add here other infos resets */
}

/* called before each strat, and before the start switch */
void strat_preinit(void)
{
    time_reset();
    interrupt_traj_reset();
    mainboard.flags |=  DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_POWER;

    strat_dump_conf();
    strat_dump_infos(__FUNCTION__);
}

/* called it just before launching the strat */
void strat_init(void)
{
    strat_reset_infos();
    strat_set_speed(SPEED_DIST_FAST, SPEED_ANGLE_FAST);
    time_reset();
    interrupt_traj_reset();

    /* used in strat_base for END_TIMER */
	if (!strat_infos.debug_step)
    	mainboard.flags |=  DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_POWER | DO_TIMER;
	else
    	mainboard.flags |=  DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_POWER;
}


/* call it after each strat */
void strat_exit(void)
{
    uint8_t flags;

    /* stop robot, disable timer */
    mainboard.flags &= ~(DO_TIMER);
    strat_hardstop();
    time_reset();

    /* disable CS, and motors */
#ifndef HOST_VERSION
    IRQ_LOCK(flags);
    mainboard.flags &= ~(DO_CS);
    IRQ_UNLOCK(flags);

    /* TODO: stop motors */
    //dac_mc_set(MOTOR_LEFT, 0);
    //dac_mc_set(MOTOR_RIGHT, 0);
#endif

	strat_infos.match_ends = 1;
}

/* called periodically */
void strat_event(void *dummy) {

  /* limit speed when opponent are close */
  //strat_limit_speed();

}


/* dump state (every 5 s max) XXX */
#define DUMP_RATE_LIMIT(dump, last_print)        \
    do {                        \
        if (time_get_s() - last_print > 5) {    \
            dump();                \
            last_print = time_get_s();    \
        }                    \
    } while (0)


#define ERROUT(e) do {\
        err = e;             \
        goto end;         \
    } while(0)

/* Strat main loop */
uint8_t strat_main(void)
{
    //uint8_t err;
    strat_limit_speed_enable ();

    /* TODO strat */

    strat_exit();
    return 0;
}



#endif /* !HOST_VERSION_OA_TEST */
