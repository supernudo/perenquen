/*
 *  Copyright Droids Corporation (2009)
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
 *  Olivier MATZ <zer0@droids-corp.org>
 */

/*
 *  Copyright Javier Baliñas Santos (2018)
 *  Javier Balias Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  commands.c,v 1.8 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdlib.h>
#include <aversive/pgmspace.h>
#include <parse.h>

#ifndef HOST_VERSION

#define COMPILE_COMMANDS_GEN
#define COMPILE_COMMANDS_CS
#define COMPILE_COMMANDS_MAINBOARD
//#define COMPILE_COMMANDS_MAINBOARD_OPTIONALS
#define COMPILE_COMMANDS_TRAJ

#else /* HOST_VERSION */
#define COMPILE_COMMANDS_GEN
#define COMPILE_COMMANDS_CS
#define COMPILE_COMMANDS_MAINBOARD
#define COMPILE_COMMANDS_MAINBOARD_OPTIONALS
#define COMPILE_COMMANDS_TRAJ
#endif

/* commands_gen.c */
#ifdef COMPILE_COMMANDS_GEN
#ifndef HOST_VERSION
#include "./commands_gen.c"
#endif
extern parse_pgm_inst_t cmd_reset;
extern parse_pgm_inst_t cmd_encoders;
extern parse_pgm_inst_t cmd_motors;
extern parse_pgm_inst_t cmd_sensors;
extern parse_pgm_inst_t cmd_log;
extern parse_pgm_inst_t cmd_log_show;
extern parse_pgm_inst_t cmd_log_type;
extern parse_pgm_inst_t cmd_scheduler;
#endif /* COMPILE_COMMANDS_GEN */

/* commands_cs.c */
#ifdef COMPILE_COMMANDS_CS

#ifndef HOST_VERSION
#include "./commands_cs.c"
#endif

extern parse_pgm_inst_t cmd_gain;
extern parse_pgm_inst_t cmd_gain_show;
extern parse_pgm_inst_t cmd_speed;
extern parse_pgm_inst_t cmd_speed_show;
extern parse_pgm_inst_t cmd_derivate_filter;
extern parse_pgm_inst_t cmd_derivate_filter_show;
extern parse_pgm_inst_t cmd_consign;
extern parse_pgm_inst_t cmd_maximum;
extern parse_pgm_inst_t cmd_maximum_show;
extern parse_pgm_inst_t cmd_quadramp;
extern parse_pgm_inst_t cmd_quadramp_show;
extern parse_pgm_inst_t cmd_cs_status;
extern parse_pgm_inst_t cmd_blocking_i;
extern parse_pgm_inst_t cmd_blocking_i_show;

#endif /* COMPILE_COMMANDS_CS */

/* commands_mainboard.c */
#ifdef COMPILE_COMMANDS_MAINBOARD

#ifndef HOST_VERSION
#include "./commands_mainboard.c"
#endif

extern parse_pgm_inst_t cmd_event;
extern parse_pgm_inst_t cmd_init;
extern parse_pgm_inst_t cmd_start;
extern parse_pgm_inst_t cmd_color;
#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS
extern parse_pgm_inst_t cmd_interact;
extern parse_pgm_inst_t cmd_rs;
#ifdef TRAJECTORY_MANAGER_V3
extern parse_pgm_inst_t cmd_clitoid;
#endif /* TRAJECTORY_MANAGER_V3 */
extern parse_pgm_inst_t cmd_time_monitor;
extern parse_pgm_inst_t cmd_sleep;
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/
#endif /* COMPILE_COMMANDS_MAINBOARD */



/* commands_traj.c */
#ifdef COMPILE_COMMANDS_TRAJ

#ifndef HOST_VERSION
#include "./commands_traj.c"
#endif

extern parse_pgm_inst_t cmd_traj_speed;
extern parse_pgm_inst_t cmd_traj_speed_show;

#ifdef TRAJECTORY_MANAGER_V3
extern parse_pgm_inst_t cmd_traj_acc;
extern parse_pgm_inst_t cmd_traj_acc_show;
extern parse_pgm_inst_t cmd_circle_coef;
extern parse_pgm_inst_t cmd_circle_coef_show;
#endif /* TRAJECTORY_MANAGER_V3  */
extern parse_pgm_inst_t cmd_trajectory;
extern parse_pgm_inst_t cmd_trajectory_show;
extern parse_pgm_inst_t cmd_rs_gains;
extern parse_pgm_inst_t cmd_rs_gains_show;
extern parse_pgm_inst_t cmd_track;
extern parse_pgm_inst_t cmd_track_show;
extern parse_pgm_inst_t cmd_centrifugal;
extern parse_pgm_inst_t cmd_centrifugal_show;
extern parse_pgm_inst_t cmd_pt_list;
extern parse_pgm_inst_t cmd_pt_list_append;
extern parse_pgm_inst_t cmd_pt_list_del;
extern parse_pgm_inst_t cmd_pt_list_show;

extern parse_pgm_inst_t cmd_goto1;
extern parse_pgm_inst_t cmd_goto2;
extern parse_pgm_inst_t cmd_goto3;
extern parse_pgm_inst_t cmd_position;
extern parse_pgm_inst_t cmd_position_set;

/* TODO 2015 */
extern parse_pgm_inst_t cmd_strat_infos;
extern parse_pgm_inst_t cmd_strat_conf;
extern parse_pgm_inst_t cmd_subtraj1;
#endif /* COMPILE_COMMANDS_TRAJ*/

/* in progmem */
parse_pgm_ctx_t main_ctx[] = {

    /* commands_gen.c */
    #ifdef COMPILE_COMMANDS_GEN
    (parse_pgm_inst_t *) & cmd_reset,
    (parse_pgm_inst_t *) & cmd_encoders,
    (parse_pgm_inst_t *) & cmd_scheduler,
    (parse_pgm_inst_t *) & cmd_motors,
    (parse_pgm_inst_t *) & cmd_sensors,
    (parse_pgm_inst_t *) & cmd_log,
    (parse_pgm_inst_t *) & cmd_log_show,
    (parse_pgm_inst_t *) & cmd_log_type,
    #endif /* COMPILE_COMMANDS_GEN */


    /* commands_cs.c */
    #ifdef COMPILE_COMMANDS_CS
    (parse_pgm_inst_t *) & cmd_gain,
    (parse_pgm_inst_t *) & cmd_gain_show,
    (parse_pgm_inst_t *) & cmd_speed,
    (parse_pgm_inst_t *) & cmd_speed_show,
    (parse_pgm_inst_t *) & cmd_consign,
    (parse_pgm_inst_t *) & cmd_derivate_filter,
    (parse_pgm_inst_t *) & cmd_derivate_filter_show,
    (parse_pgm_inst_t *) & cmd_maximum,
    (parse_pgm_inst_t *) & cmd_maximum_show,
    (parse_pgm_inst_t *) & cmd_quadramp,
    (parse_pgm_inst_t *) & cmd_quadramp_show,
    (parse_pgm_inst_t *) & cmd_cs_status,
    (parse_pgm_inst_t *) & cmd_blocking_i,
    (parse_pgm_inst_t *) & cmd_blocking_i_show,
    #endif /* COMPILE_COMMANDS_CS */

    /* commands_mainboard.c */
    #ifdef COMPILE_COMMANDS_MAINBOARD
    (parse_pgm_inst_t *) & cmd_event,
    (parse_pgm_inst_t *) & cmd_init,
    (parse_pgm_inst_t *) & cmd_start,

    #ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS
    (parse_pgm_inst_t *) & cmd_interact,
    (parse_pgm_inst_t *) & cmd_rs,
    #ifdef TRAJECTORY_MANAGER_V3
    (parse_pgm_inst_t *) & cmd_clitoid,
    #endif
    (parse_pgm_inst_t *) & cmd_time_monitor,
    //(parse_pgm_inst_t *) & cmd_strat_event,
    (parse_pgm_inst_t *) & cmd_sleep,
    #endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS */
    #endif /* COMPILE_COMMANDS_MAINBOARD */

    /* commands_traj.c */
    #ifdef COMPILE_COMMANDS_TRAJ
    (parse_pgm_inst_t *) & cmd_traj_speed,
    (parse_pgm_inst_t *) & cmd_traj_speed_show,

    #ifdef TRAJECTORY_MANAGER_V3
    (parse_pgm_inst_t *) & cmd_traj_acc,
    (parse_pgm_inst_t *) & cmd_traj_acc_show,
    (parse_pgm_inst_t *) & cmd_circle_coef,
    (parse_pgm_inst_t *) & cmd_circle_coef_show,
    #endif /* TRAJECTORY_MANAGER_V3 */
    (parse_pgm_inst_t *) & cmd_trajectory,
    (parse_pgm_inst_t *) & cmd_trajectory_show,
    (parse_pgm_inst_t *) & cmd_rs_gains,
    (parse_pgm_inst_t *) & cmd_rs_gains_show,
    (parse_pgm_inst_t *) & cmd_track,
    (parse_pgm_inst_t *) & cmd_track_show,
    (parse_pgm_inst_t *) & cmd_centrifugal,
    (parse_pgm_inst_t *) & cmd_centrifugal_show,
    (parse_pgm_inst_t *) & cmd_pt_list,
    (parse_pgm_inst_t *) & cmd_pt_list_append,
    (parse_pgm_inst_t *) & cmd_pt_list_del,
    (parse_pgm_inst_t *) & cmd_pt_list_show,
    (parse_pgm_inst_t *) & cmd_goto1,
    (parse_pgm_inst_t *) & cmd_goto2,
    (parse_pgm_inst_t *) & cmd_position,
    (parse_pgm_inst_t *) & cmd_position_set,

    /* TODO */
    (parse_pgm_inst_t *) & cmd_strat_infos,
    (parse_pgm_inst_t *) & cmd_strat_conf,
    //(parse_pgm_inst_t *) & cmd_subtraj1,
    #endif /* COMPILE_COMMANDS_TRAJ */

    NULL,
};
