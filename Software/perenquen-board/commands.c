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
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
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
//#define COMPILE_COMMANDS_GEN_OPTIONALS
//#define COMPILE_COMMANDS_CS
#define COMPILE_COMMANDS_MAINBOARD
//#define COMPILE_COMMANDS_MAINBOARD_OPTIONALS
#define COMPILE_COMMANDS_TRAJ
//#define COMPILE_COMMANDS_TRAJ_OPTIONALS

#else /* HOST_VERSION */
#define COMPILE_COMMANDS_GEN
#define COMPILE_COMMANDS_GEN_OPTIONALS
#define COMPILE_COMMANDS_CS
#define COMPILE_COMMANDS_MAINBOARD
#define COMPILE_COMMANDS_MAINBOARD_OPTIONALS
#define COMPILE_COMMANDS_TRAJ
#define COMPILE_COMMANDS_TRAJ_OPTIONALS
#endif

#define COMPILE_CODE
#ifdef COMPILE_CODE /*---------------------------------------------------------------------------------------------*/
#endif /* COMPILE_CODE --------------------------------------------------------------------------------------------*/

/* commands_gen.c */
#ifdef COMPILE_COMMANDS_GEN

#ifndef HOST_VERSION
#include "./commands_gen.c"
#endif

extern parse_pgm_inst_t cmd_reset;
//TODO extern parse_pgm_inst_t cmd_bootloader;
extern parse_pgm_inst_t cmd_encoders;
extern parse_pgm_inst_t cmd_pwm_servo;
extern parse_pgm_inst_t cmd_pwm_servo_show_range;
extern parse_pgm_inst_t cmd_dac_mc;

#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/
extern parse_pgm_inst_t cmd_adc;
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS --------------------------------*/

extern parse_pgm_inst_t cmd_sensor;
extern parse_pgm_inst_t cmd_wt11;
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
// TODO extern parse_pgm_inst_t cmd_spi_test;
extern parse_pgm_inst_t cmd_opponent;
extern parse_pgm_inst_t cmd_opponent_set;
extern parse_pgm_inst_t cmd_init;
extern parse_pgm_inst_t cmd_start;
extern parse_pgm_inst_t cmd_color;
extern parse_pgm_inst_t cmd_beacon;
extern parse_pgm_inst_t cmd_robot_2nd;
extern parse_pgm_inst_t cmd_robot_2nd_goto1;
extern parse_pgm_inst_t cmd_robot_2nd_goto2;
extern parse_pgm_inst_t cmd_robot_2nd_bt_task;
extern parse_pgm_inst_t cmd_robot_2nd_bt_task2;
extern parse_pgm_inst_t cmd_slavedspic;

#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/
extern parse_pgm_inst_t cmd_sensor_robot;
extern parse_pgm_inst_t cmd_interact;
extern parse_pgm_inst_t cmd_rs;
#ifdef TRAJECTORY_MANAGER_V3
extern parse_pgm_inst_t cmd_clitoid;
#endif /* TRAJECTORY_MANAGER_V3 */
extern parse_pgm_inst_t cmd_time_monitor;
extern parse_pgm_inst_t cmd_sleep;
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/

#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/

extern parse_pgm_inst_t cmd_popcorn_system;
extern parse_pgm_inst_t cmd_stands_system;
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/


#endif /* COMPILE_COMMANDS_MAINBOARD */



/* commands_traj.c */
#ifdef COMPILE_COMMANDS_TRAJ

#ifndef HOST_VERSION
#include "./commands_traj.c"
#endif

extern parse_pgm_inst_t cmd_traj_speed;
extern parse_pgm_inst_t cmd_traj_speed_show;

#ifdef COMPILE_COMMANDS_TRAJ_OPTIONALS /*-------------------------------------*/
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
#endif /* COMPILE_COMMANDS_TRAJ_OPTIONALS ------------------------------------*/

extern parse_pgm_inst_t cmd_goto1;
extern parse_pgm_inst_t cmd_goto2;
extern parse_pgm_inst_t cmd_goto3;
extern parse_pgm_inst_t cmd_position;
extern parse_pgm_inst_t cmd_position_set;

/* TODO 2015 */
extern parse_pgm_inst_t cmd_strat_db;
extern parse_pgm_inst_t cmd_strat_infos;
extern parse_pgm_inst_t cmd_strat_conf;
extern parse_pgm_inst_t cmd_strat_conf2;
extern parse_pgm_inst_t cmd_strat_conf3;
extern parse_pgm_inst_t cmd_subtraj1;
extern parse_pgm_inst_t cmd_subtraj2;

//extern parse_pgm_inst_t cmd_workonzone;
//extern parse_pgm_inst_t cmd_homologation;

#endif /* COMPILE_COMMANDS_TRAJ*/

/* in progmem */
parse_pgm_ctx_t main_ctx[] = {

#ifdef COMPILE_COMMANDS_GEN

#ifdef COMPILE_COMMANDS_GEN_OPTIONALS /*--------------------------------*/
    /* commands_gen.c */
    (parse_pgm_inst_t *) & cmd_reset,
    //	(parse_pgm_inst_t *)&cmd_bootloader,
#endif /* COMPILE_COMMANDS_GEN_OPTIONALS --------------------------------*/

    (parse_pgm_inst_t *) & cmd_encoders,

#ifdef COMPILE_COMMANDS_GEN_OPTIONALS /*--------------------------------*/
    (parse_pgm_inst_t *) & cmd_scheduler,
    (parse_pgm_inst_t *) & cmd_pwm_servo,
    (parse_pgm_inst_t *) & cmd_pwm_servo_show_range,
    (parse_pgm_inst_t *) & cmd_dac_mc,
#endif

#ifdef COMPILE_COMMANDS_GEN_OPTIONALS /*--------------------------------*/
    (parse_pgm_inst_t *) & cmd_adc,
#endif /* COMPILE_COMMANDS_GEN_OPTIONALS --------------------------------*/

#ifdef COMPILE_COMMANDS_GEN_OPTIONALS /*--------------------------------*/
    (parse_pgm_inst_t *) & cmd_sensor,
    (parse_pgm_inst_t *) & cmd_wt11,

    (parse_pgm_inst_t *) & cmd_log,
    (parse_pgm_inst_t *) & cmd_log_show,
    (parse_pgm_inst_t *) & cmd_log_type,

#endif /* COMPILE_COMMANDS_GEN_OPTIONALS --------------------------------*/


#endif /* COMPILE_COMMANDS_GEN */

#ifdef COMPILE_COMMANDS_CS

    /* commands_cs.c */
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

#ifdef COMPILE_COMMANDS_MAINBOARD

    /* commands_mainboard.c */
    (parse_pgm_inst_t *) & cmd_event,
    (parse_pgm_inst_t *) & cmd_opponent,
#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/
    (parse_pgm_inst_t *) & cmd_opponent_set,
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/
    (parse_pgm_inst_t *) & cmd_init,
    (parse_pgm_inst_t *) & cmd_start,
    (parse_pgm_inst_t *) & cmd_color,
    (parse_pgm_inst_t *) & cmd_beacon,

#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/
    (parse_pgm_inst_t *) & cmd_robot_2nd,
    (parse_pgm_inst_t *) & cmd_robot_2nd_goto1,
    (parse_pgm_inst_t *) & cmd_robot_2nd_goto2,
	(parse_pgm_inst_t *) & cmd_robot_2nd_bt_task,
	(parse_pgm_inst_t *) & cmd_robot_2nd_bt_task2,
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/

    (parse_pgm_inst_t *) & cmd_slavedspic,
    (parse_pgm_inst_t *) & cmd_sensor_robot,

#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/
    (parse_pgm_inst_t *) & cmd_interact,
    (parse_pgm_inst_t *) & cmd_rs,
#ifdef TRAJECTORY_MANAGER_V3
    (parse_pgm_inst_t *) & cmd_clitoid,
#endif
    (parse_pgm_inst_t *) & cmd_time_monitor,
   // (parse_pgm_inst_t *) & cmd_strat_event,
    (parse_pgm_inst_t *) & cmd_sleep,
#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/
 
//#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/
    (parse_pgm_inst_t *) & cmd_popcorn_system,
    (parse_pgm_inst_t *) & cmd_stands_system,
//#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/

#endif /* COMPILE_COMMANDS_MAINBOARD */

#ifdef COMPILE_COMMANDS_TRAJ

    /* commands_traj.c */
    (parse_pgm_inst_t *) & cmd_traj_speed,
    (parse_pgm_inst_t *) & cmd_traj_speed_show,

#ifdef COMPILE_COMMANDS_TRAJ_OPTIONALS /*-------------------------------------*/
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
#endif /* COMPILE_COMMANDS_TRAJ_OPTIONALS ------------------------------------*/

    (parse_pgm_inst_t *) & cmd_goto1,
    (parse_pgm_inst_t *) & cmd_goto2,
    (parse_pgm_inst_t *) & cmd_position,
    (parse_pgm_inst_t *) & cmd_position_set,

    /* TODO 2015 */
    (parse_pgm_inst_t *) & cmd_strat_infos,
    (parse_pgm_inst_t *) & cmd_strat_conf,
#ifdef COMPILE_COMMANDS_TRAJ_OPTIONALS /*-------------------------------------*/
    (parse_pgm_inst_t *) & cmd_strat_conf2,
    (parse_pgm_inst_t *) & cmd_strat_conf3,
#endif /* COMPILE_COMMANDS_TRAJ_OPTIONALS ------------------------------------*/
    (parse_pgm_inst_t *) & cmd_subtraj1,
    (parse_pgm_inst_t *) & cmd_subtraj2,

    //(parse_pgm_inst_t *) & cmd_gotozone,
    //(parse_pgm_inst_t *) & cmd_workonzone,
    //(parse_pgm_inst_t *) & cmd_homologation,


#endif /* COMPILE_COMMANDS_TRAJ */

    NULL,
};
