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
 *  commands_mainboard.c,v 1.8 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <string.h>

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

#ifdef HOST_VERSION
#include <hostsim.h>
#endif

#include <rdline.h>
#include <parse.h>
#include <parse_string.h>
#include <parse_num.h>



#include "main.h"
#include "sensor.h"
#include "cmdline.h"
#include "strat.h"
#include "strat_utils.h"
#include "strat_base.h"

#include "actuator.h"
//

#include "robotsim.h"
#include "strat_base.h"

#include "strat_utils.h"

uint8_t strat_obstacle(void)
{
  #define S_FRONT_OBSTACLE_VALUE 500
  sensor_adc_do_read(S_ADC_FRONT_LEFT);
  sensor_adc_do_read(S_ADC_FRONT_RIGHT);
  return (sensor_adc_get_value(S_ADC_FRONT_LEFT) > S_FRONT_OBSTACLE_VALUE &&
          sensor_adc_get_value(S_ADC_FRONT_RIGHT) > S_FRONT_OBSTACLE_VALUE);
}

void strat_follow_wall(uint8_t side, int32_t gain)
{
  #define STATE_CALIB         0
  #define STATE_WAITING_START 1
  #define STATE_GO_FORWARD    2
  #define STATE_WALL_TRACKER  3
  #define STATE_TURN          4
  #define STATE_TURN_END      5

  #define S_FRONT_START_VALUE     200
  #define S_FRONT_END_TURN_VALUE  150

  uint8_t state = STATE_CALIB;
  uint16_t calib_diag_left=0, calib_diag_right=0;
  int16_t error;
  float angle = 0.0;
  uint8_t err;
  uint8_t running = 1;
  uint8_t sensor;


  interrupt_traj_reset();

  while (running)
  {
    switch(state) {
      case STATE_CALIB:
        sensor_adc_do_read(S_ADC_DIAG_LEFT);
        sensor_adc_do_read(S_ADC_DIAG_RIGHT);
        calib_diag_left = sensor_adc_get_value(S_ADC_DIAG_LEFT);
        calib_diag_right = sensor_adc_get_value(S_ADC_DIAG_RIGHT);
        DEBUG(E_USER_STRAT, "calib left %d\n\r", calib_diag_left);
        DEBUG(E_USER_STRAT, "calib right %d\n\r", calib_diag_right);
        state = STATE_WAITING_START;
        break;

      case STATE_WAITING_START:
        sensor_adc_do_read(S_ADC_FRONT_LEFT);
        sensor_adc_do_read(S_ADC_FRONT_RIGHT);
        DEBUG(E_USER_STRAT, "Waiting START...\n\r");
        while(sensor_adc_get_value(S_ADC_FRONT_LEFT) < S_FRONT_START_VALUE &&
              sensor_adc_get_value(S_ADC_FRONT_RIGHT) < S_FRONT_START_VALUE)
        {
          sensor_adc_do_read(S_ADC_FRONT_LEFT);
          sensor_adc_do_read(S_ADC_FRONT_RIGHT);
          time_wait_ms(100);
        }
        while(sensor_adc_get_value(S_ADC_FRONT_LEFT) > S_FRONT_START_VALUE ||
              sensor_adc_get_value(S_ADC_FRONT_RIGHT) > S_FRONT_START_VALUE)
        {
          sensor_adc_do_read(S_ADC_FRONT_LEFT);
          sensor_adc_do_read(S_ADC_FRONT_RIGHT);
          time_wait_ms(100);
        }

        DEBUG(E_USER_STRAT, "GO!!!!\n\r");
        state = STATE_GO_FORWARD;
        break;

      case STATE_GO_FORWARD:
        trajectory_d_rel(&mainboard.traj, 30000);
        DEBUG(E_USER_STRAT, "Go forward\n\r");
        state = STATE_WALL_TRACKER;
        break;

      case STATE_WALL_TRACKER:

        /* Wall tracker */
        //time_wait_ms(100);
        if (side == SIDE_LEFT) {
          sensor_adc_do_read(S_ADC_DIAG_LEFT);

          error = calib_diag_left - sensor_adc_get_value(S_ADC_DIAG_LEFT);
          angle = error*gain/1024;
          DEBUG(E_USER_STRAT, "LEFT error %d, angle %f\n\r", error, angle);

          trajectory_only_a_rel(&mainboard.traj, (int32_t)angle);
        }
        else {
          sensor_adc_do_read(S_ADC_DIAG_RIGHT);

          error = calib_diag_right - sensor_adc_get_value(S_ADC_DIAG_RIGHT);
          angle = -error*gain/1024;
          DEBUG(E_USER_STRAT, "RIGHT error %d, angle %f\n\r", error, angle);

          trajectory_only_a_rel(&mainboard.traj, (int32_t)angle);
        }

        /* End traj */
        err = test_traj_end(END_TRAJ|END_OBSTACLE|END_INTR);
        if(err == END_TRAJ) {
          DEBUG(E_USER_STRAT, "traj returns END_TRAJ\n\r");
          state = STATE_GO_FORWARD;
        }
        else if(err == END_OBSTACLE) {
          DEBUG(E_USER_STRAT, "traj returns END_OBSTACLE\n\r");
          state = STATE_TURN;
          //running = 0;
        }
        else if(err == END_INTR) {
          DEBUG(E_USER_STRAT, "traj returns END_INTR\n\r");
          //running = 0;
        }
        break;

      case STATE_TURN:
        trajectory_a_rel(&mainboard.traj, (side==SIDE_LEFT? -270:270));
        DEBUG(E_USER_STRAT, "Turn %d start\n\r", (side==SIDE_LEFT? -270:270));
        state = STATE_TURN_END;
        break;

      case STATE_TURN_END:
        DEBUG(E_USER_STRAT, "Waiting turn end...\n\r");
        sensor = (side==SIDE_LEFT? S_ADC_FRONT_LEFT : S_ADC_FRONT_RIGHT);
        sensor_adc_do_read(sensor);
        err = 0;
        while(sensor_adc_get_value(sensor) > S_FRONT_END_TURN_VALUE && err == 0) {
          sensor_adc_do_read(sensor);
          err = test_traj_end(END_TRAJ);
        }

        if(err == END_TRAJ) {
          DEBUG(E_USER_STRAT, "traj returns END_TRAJ\n\r");
          state = STATE_TURN;
          break;
        }

        trajectory_hardstop(&mainboard.traj);
        //time_wait_ms(100);
        DEBUG(E_USER_STRAT, "Turn end\n\r");
        state = STATE_GO_FORWARD;
        break;

      default:
        break;
    }
  }
}

struct cmd_event_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
    fixed_string_t arg2;
};

/* function called when cmd_event is parsed successfully */
static void cmd_event_parsed(void *parsed_result, void *data)
{
    uint16_t bit = 0;

    struct cmd_event_result * res = parsed_result;

    if (!strcmp_P(res->arg1, PSTR("all")))
    {
        bit = DO_ENCODERS | DO_CS | DO_RS | DO_POS | DO_BD | DO_TIMER | DO_POWER | DO_TM_DATA;
        if (!strcmp_P(res->arg2, PSTR("on")))
            mainboard.flags |= bit;
        else if (!strcmp_P(res->arg2, PSTR("off")))
            mainboard.flags &= bit;
        else
        { /* show */
            printf_P(PSTR("encoders is %s\r\n"),
                     (DO_ENCODERS & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("cs is %s\r\n"),
                     (DO_CS & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("rs is %s\r\n"),
                     (DO_RS & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("pos is %s\r\n"),
                     (DO_POS & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("bd is %s\r\n"),
                     (DO_BD & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("timer is %s\r\n"),
                     (DO_TIMER & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("power is %s\r\n"),
                     (DO_POWER & mainboard.flags) ? "on" : "off");
            printf_P(PSTR("tm is %s\r\n"),
                     (DO_TM_DATA & mainboard.flags) ? "on" : "off");
        }
        return;
    }

    if (!strcmp_P(res->arg1, PSTR("encoders")))
        bit = DO_ENCODERS;
    else if (!strcmp_P(res->arg1, PSTR("cs")))
    {
        strat_hardstop();
        bit = DO_CS;
    }
    else if (!strcmp_P(res->arg1, PSTR("rs")))
        bit = DO_RS;
    else if (!strcmp_P(res->arg1, PSTR("pos")))
        bit = DO_POS;
    else if (!strcmp_P(res->arg1, PSTR("bd")))
        bit = DO_BD;
    else if (!strcmp_P(res->arg1, PSTR("timer")))
    {
        time_reset();
        bit = DO_TIMER;
    }
    else if (!strcmp_P(res->arg1, PSTR("power")))
        bit = DO_POWER;
    else if (!strcmp_P(res->arg1, PSTR("tm")))
        bit = DO_TM_DATA;

    if (!strcmp_P(res->arg2, PSTR("on")))
        mainboard.flags |= bit;
    else if (!strcmp_P(res->arg2, PSTR("off")))
    {
        if (!strcmp_P(res->arg1, PSTR("cs")))
        {
#ifdef HOST_VERSION
            robotsim_pwm(MOTOR_LEFT, 0);
            robotsim_pwm(MOTOR_RIGHT, 0);
#else
            hspwm_set_pwm(MOTOR_LEFT, 0);
          	hspwm_set_pwm(MOTOR_RIGHT, 0);
#endif
        }
        mainboard.flags &= (~bit);
    }
    printf_P(PSTR("%s is %s\r\n"), res->arg1,
             (bit & mainboard.flags) ? "on" : "off");
}

prog_char str_event_arg0[] = "event";
parse_pgm_token_string_t cmd_event_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg0, str_event_arg0);
prog_char str_event_arg1[] = "all#encoders#cs#rs#pos#bd#timer#power#tm";
parse_pgm_token_string_t cmd_event_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg1, str_event_arg1);
prog_char str_event_arg2[] = "on#off#show";
parse_pgm_token_string_t cmd_event_arg2 = TOKEN_STRING_INITIALIZER(struct cmd_event_result, arg2, str_event_arg2);

prog_char help_event[] = "Enable/disable events";
parse_pgm_inst_t cmd_event = {
    .f = cmd_event_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_event,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_event_arg0,
        (prog_void *) & cmd_event_arg1,
        (prog_void *) & cmd_event_arg2,
        NULL,
    },
};

/**********************************************************/
/* Init match */

/* from commands_traj.c */
//void auto_position(void);

/* this structure is filled when cmd_init is parsed successfully */
struct cmd_init_result
{
    fixed_string_t arg0;
    fixed_string_t color;
};


/* function called when cmd_init is parsed successfully */
static void cmd_init_parsed(void *parsed_result, void *data)
{
    //struct cmd_init_result *res = parsed_result;

	/* autopos main robot */
	//auto_position();

	time_wait_ms (200);

	printf ("Done\n\r");
}

prog_char str_init_arg0[] = "init";
parse_pgm_token_string_t cmd_init_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_init_result, arg0, str_init_arg0);
prog_char str_init_color[] = "left#righ";
parse_pgm_token_string_t cmd_init_color = TOKEN_STRING_INITIALIZER(struct cmd_init_result, color, str_init_color);

prog_char help_init[] = "Init the robots";
parse_pgm_inst_t cmd_init = {
    .f = cmd_init_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_init,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_init_arg0,
        (prog_void *) & cmd_init_color,
        NULL,
    },
};



/**********************************************************/
/* Start */

/* this structure is filled when cmd_start is parsed successfully */
struct cmd_start_result
{
    fixed_string_t arg0;
    fixed_string_t strategy;
    int32_t arg2;
    //fixed_string_t debug;
};

/* function called when cmd_start is parsed successfully */
static void cmd_start_parsed(void *parsed_result, void *data)
{
    struct cmd_start_result *res = parsed_result;

    if (!strcmp_P(res->strategy, PSTR("left")))
      strat_follow_wall(SIDE_LEFT, res->arg2);
    else
      strat_follow_wall(SIDE_RIGHT, res->arg2);

    // OK speed 1000 gain 100
    // OK speed 1500 gain 200

    //uint8_t old_level = gen.log_level;
    //int8_t c;

    // TODO
    //gen.logs[NB_LOGS] = E_USER_STRAT;
    //if (!strcmp_P(res->debug, PSTR("debug")))
    //{
    //    strat_infos.dump_enabled = 1;
    //    gen.log_level = 5;
    //}
    //else if (!strcmp_P(res->debug, PSTR("step_debug")))
    //{
    //    strat_infos.dump_enabled = 1;
    //    strat_infos.debug_step = 1;
    //    gen.log_level = 5;
    //}
    //else
    //{
    //    strat_infos.dump_enabled = 0;
    //    gen.log_level = 0;
    //}

    //strat_dump_conf();

    //strat_start();

    //gen.logs[NB_LOGS] = 0;
    //gen.log_level = old_level;
}

prog_char str_start_arg0[] = "start";
parse_pgm_token_string_t cmd_start_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_start_result, arg0, str_start_arg0);
prog_char str_start_strategy[] = "left#right";
parse_pgm_token_string_t cmd_start_strategy = TOKEN_STRING_INITIALIZER(struct cmd_start_result, strategy, str_start_strategy);
parse_pgm_token_num_t cmd_start_arg2 = TOKEN_NUM_INITIALIZER(struct cmd_start_result, arg2, INT32);

//prog_char str_start_debug[] = "debug#step_debug#match";
//parse_pgm_token_string_t cmd_start_debug = TOKEN_STRING_INITIALIZER(struct cmd_start_result, debug, str_start_debug);

prog_char help_start[] = "Start the robot";
parse_pgm_inst_t cmd_start = {
    .f = cmd_start_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_start,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_start_arg0,
        (prog_void *) & cmd_start_strategy,
        (prog_void *) & cmd_start_arg2,
        //(prog_void *) & cmd_start_debug,
        NULL,
    },
};



#ifdef COMPILE_COMMANDS_MAINBOARD_OPTIONALS /*--------------------------------*/

/**********************************************************/
/* Interact */

/* this structure is filled when cmd_interact is parsed successfully */
struct cmd_interact_result
{
    fixed_string_t arg0;
};

static void print_cs(void)
{
    printf_P(PSTR("cons_d=% .8"PRIi32" cons_a=% .8"PRIi32" fil_d=% .8"PRIi32" fil_a=% .8"PRIi32" "
                  "err_d=% .8"PRIi32" err_a=% .8"PRIi32" out_d=% .8"PRIi32" out_a=% .8"PRIi32"\r\n"),
             cs_get_consign(&mainboard.distance.cs),
             cs_get_consign(&mainboard.angle.cs),
             cs_get_filtered_consign(&mainboard.distance.cs),
             cs_get_filtered_consign(&mainboard.angle.cs),
             cs_get_error(&mainboard.distance.cs),
             cs_get_error(&mainboard.angle.cs),
             cs_get_out(&mainboard.distance.cs),
             cs_get_out(&mainboard.angle.cs));
}

static void print_pos(void)
{
    printf_P(PSTR("x=% .8d y=% .8d a=% .8d\r\n"),
             position_get_x_s16(&mainboard.pos),
             position_get_y_s16(&mainboard.pos),
             position_get_a_deg_s16(&mainboard.pos));
}

static void print_time(void)
{
    printf_P(PSTR("time %d\r\n"), (int16_t) time_get_s());
}

static void print_sensors(void)
{
#ifdef notyet
    if (sensor_start_switch())
        printf_P(PSTR("Start switch | "));
    else
        printf_P(PSTR("             | "));

    if (IR_DISP_SENSOR())
        printf_P(PSTR("IR disp | "));
    else
        printf_P(PSTR("        | "));

    printf_P(PSTR("\r\n"));
#endif
}

static void print_pid(void)
{
    printf_P(PSTR("P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32" | "
                  "P=% .8"PRIi32" I=% .8"PRIi32" D=% .8"PRIi32" out=% .8"PRIi32"\r\n"),
             pid_get_value_in(&mainboard.distance.pid) * pid_get_gain_P(&mainboard.distance.pid),
             pid_get_value_I(&mainboard.distance.pid) * pid_get_gain_I(&mainboard.distance.pid),
             pid_get_value_D(&mainboard.distance.pid) * pid_get_gain_D(&mainboard.distance.pid),
             pid_get_value_out(&mainboard.distance.pid),
             pid_get_value_in(&mainboard.angle.pid) * pid_get_gain_P(&mainboard.angle.pid),
             pid_get_value_I(&mainboard.angle.pid) * pid_get_gain_I(&mainboard.angle.pid),
             pid_get_value_D(&mainboard.angle.pid) * pid_get_gain_D(&mainboard.angle.pid),
             pid_get_value_out(&mainboard.angle.pid));
}

#define PRINT_POS       (1<<0)
#define PRINT_PID       (1<<1)
#define PRINT_CS        (1<<2)
#define PRINT_SENSORS   (1<<3)
#define PRINT_TIME      (1<<4)
#define PRINT_BLOCKING  (1<<5)

static void cmd_interact_parsed(void * parsed_result, void * data)
{
    int c;
    int8_t cmd;
    uint8_t print = 0;
    struct vt100 vt100;

    vt100_init(&vt100);

    printf_P(PSTR("Display debugs:\r\n"
                  "  1:pos\r\n"
                  "  2:pid\r\n"
                  "  3:cs\r\n"
                  "  4:sensors\r\n"
                  "  5:time\r\n"
                  /* "  6:blocking\r\n" */
                  "Commands:\r\n"
                  "  arrows:move\r\n"
                  "  space:stop\r\n"
                  "  q:quit\r\n"));

    /* stop motors */
    mainboard.flags &= (~DO_CS);
    motor_pwm_set_and_save(MOTOR_LEFT, 0);
    motor_pwm_set_and_save(MOTOR_RIGHT, 0);

    while (1)
    {
        if (print & PRINT_POS)
        {
            print_pos();
        }

        if (print & PRINT_PID)
        {
            print_pid();
        }

        if (print & PRINT_CS)
        {
            print_cs();
        }

        if (print & PRINT_SENSORS)
        {
            print_sensors();
        }

        if (print & PRINT_TIME)
        {
            print_time();
        }
        /* 		if (print & PRINT_BLOCKING) { */
        /* 			printf_P(PSTR("%s %s blocking=%d\r\n"),  */
        /* 				 mainboard.blocking ? "BLOCK1":"      ", */
        /* 				 rs_is_blocked(&mainboard.rs) ? "BLOCK2":"      ", */
        /* 				 rs_get_blocking(&mainboard.rs)); */
        /* 		} */

        c = cmdline_getchar();
        if (c == -1)
        {
            wait_ms(10);
            continue;
        }
        cmd = vt100_parser(&vt100, c);
        if (cmd == -2)
        {
            wait_ms(10);
            continue;
        }

        if (cmd == -1)
        {
            switch (c)
            {
            case '1': print ^= PRINT_POS;
                break;
            case '2': print ^= PRINT_PID;
                break;
            case '3': print ^= PRINT_CS;
                break;
            case '4': print ^= PRINT_SENSORS;
                break;
            case '5': print ^= PRINT_TIME;
                break;
            case '6': print ^= PRINT_BLOCKING;
                break;

            case 'q':
                if (mainboard.flags & DO_CS)
                    strat_hardstop();
                motor_pwm_set_and_save(MOTOR_LEFT, 0);
                motor_pwm_set_and_save(MOTOR_RIGHT, 0);
                return;
            case ' ':
                motor_pwm_set_and_save(MOTOR_LEFT, 0);
                motor_pwm_set_and_save(MOTOR_RIGHT, 0);
                break;
            default:
                break;
            }
        }
        else
        {
#ifdef HOST_VERSION
#define PWM_INTERACT 1200
#else
#define PWM_INTERACT 1200
#endif
            switch (cmd)
            {
            case KEY_UP_ARR:
                motor_pwm_set_and_save(MOTOR_LEFT, PWM_INTERACT);
                motor_pwm_set_and_save(MOTOR_RIGHT, PWM_INTERACT);
                break;
            case KEY_LEFT_ARR:
                motor_pwm_set_and_save(MOTOR_LEFT, -PWM_INTERACT);
                motor_pwm_set_and_save(MOTOR_RIGHT, PWM_INTERACT);
                break;
            case KEY_DOWN_ARR:
                motor_pwm_set_and_save(MOTOR_LEFT, -PWM_INTERACT);
                motor_pwm_set_and_save(MOTOR_RIGHT, -PWM_INTERACT);
                break;
            case KEY_RIGHT_ARR:
                motor_pwm_set_and_save(MOTOR_LEFT, PWM_INTERACT);
                motor_pwm_set_and_save(MOTOR_RIGHT, -PWM_INTERACT);
                break;
            }
        }
        wait_ms(10);
    }
}

prog_char str_interact_arg0[] = "interact";
parse_pgm_token_string_t cmd_interact_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_interact_result, arg0, str_interact_arg0);

prog_char help_interact[] = "Interactive mode";
parse_pgm_inst_t cmd_interact = {
    .f = cmd_interact_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_interact,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_interact_arg0,
        NULL,
    },
};



/**********************************************************/
/* Rs tests */

/* this structure is filled when cmd_rs is parsed successfully */
struct cmd_rs_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_rs is parsed successfully */
static void cmd_rs_parsed(void *parsed_result, void *data)
{
    //	struct cmd_rs_result *res = parsed_result;
    do
    {
        printf_P(PSTR("angle cons=% .6"PRIi32" in=% .6"PRIi32" out=% .6"PRIi32" / "),
                 cs_get_consign(&mainboard.angle.cs),
                 cs_get_filtered_feedback(&mainboard.angle.cs),
                 cs_get_out(&mainboard.angle.cs));
        printf_P(PSTR("distance cons=% .6"PRIi32" in=% .6"PRIi32" out=% .6"PRIi32" / "),
                 cs_get_consign(&mainboard.distance.cs),
                 cs_get_filtered_feedback(&mainboard.distance.cs),
                 cs_get_out(&mainboard.distance.cs));
        printf_P(PSTR("l=% .4"PRIi32" r=% .4"PRIi32"\r\n"), mainboard.motor_pwm_left,
                 mainboard.motor_pwm_right);
        wait_ms(100);
    }
    while (!cmdline_keypressed());
}

prog_char str_rs_arg0[] = "rs";
parse_pgm_token_string_t cmd_rs_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg0, str_rs_arg0);
prog_char str_rs_arg1[] = "show";
parse_pgm_token_string_t cmd_rs_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_rs_result, arg1, str_rs_arg1);

prog_char help_rs[] = "Show rs (robot system) values";
parse_pgm_inst_t cmd_rs = {
    .f = cmd_rs_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_rs,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_rs_arg0,
        (prog_void *) & cmd_rs_arg1,
        NULL,
    },
};

#ifdef TRAJECTORY_MANAGER_V3
/**********************************************************/
/* Clitoid */

/* this structure is filled when cmd_clitoid is parsed successfully */
struct cmd_clitoid_result
{
    fixed_string_t arg0;
    float alpha_deg;
    float beta_deg;
    float R_mm;
    float Vd;
    float Amax;
    float d_inter_mm;
};

/* function called when cmd_test is parsed successfully */
static void cmd_clitoid_parsed(void *parsed_result, void *data)
{
    struct cmd_clitoid_result *res = parsed_result;
    /* 	clitoid(res->alpha_deg, res->beta_deg, res->R_mm, */
    /* 		res->Vd, res->Amax, res->d_inter_mm); */
    double x = position_get_x_double(&mainboard.pos);
    double y = position_get_y_double(&mainboard.pos);
    double a = position_get_a_rad_double(&mainboard.pos);

    strat_set_speed(res->Vd, SPEED_ANGLE_FAST);
    trajectory_clitoid(&mainboard.traj, x, y, a, 150.,
                       res->alpha_deg, res->beta_deg, res->R_mm,
                       res->d_inter_mm);
}

prog_char str_clitoid_arg0[] = "clitoid";
parse_pgm_token_string_t cmd_clitoid_arg0 =
        TOKEN_STRING_INITIALIZER(struct cmd_clitoid_result,
                                 arg0, str_clitoid_arg0);
parse_pgm_token_num_t cmd_clitoid_alpha_deg =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              alpha_deg, FLOAT);
parse_pgm_token_num_t cmd_clitoid_beta_deg =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              beta_deg, FLOAT);
parse_pgm_token_num_t cmd_clitoid_R_mm =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              R_mm, FLOAT);
parse_pgm_token_num_t cmd_clitoid_Vd =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              Vd, FLOAT);
parse_pgm_token_num_t cmd_clitoid_Amax =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              Amax, FLOAT);
parse_pgm_token_num_t cmd_clitoid_d_inter_mm =
        TOKEN_NUM_INITIALIZER(struct cmd_clitoid_result,
                              d_inter_mm, FLOAT);

prog_char help_clitoid[] = "do a clitoid (alpha, beta, R, Vd, Amax, d_inter)";
parse_pgm_inst_t cmd_clitoid = {
    .f = cmd_clitoid_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_clitoid,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_clitoid_arg0,
        (prog_void *) & cmd_clitoid_alpha_deg,
        (prog_void *) & cmd_clitoid_beta_deg,
        (prog_void *) & cmd_clitoid_R_mm,
        (prog_void *) & cmd_clitoid_Vd,
        (prog_void *) & cmd_clitoid_Amax,
        (prog_void *) & cmd_clitoid_d_inter_mm,
        NULL,
    },
};
#endif /* TRAJECTORY_MANAGER_V3 */
/**********************************************************/
/* Time_Monitor */

/* this structure is filled when cmd_time_monitor is parsed successfully */
struct cmd_time_monitor_result
{
    fixed_string_t arg0;
    fixed_string_t arg1;
};

/* function called when cmd_time_monitor is parsed successfully */
static void cmd_time_monitor_parsed(void *parsed_result, void *data)
{
#ifndef HOST_VERSION
    /*
            struct cmd_time_monitor_result *res = parsed_result;
            uint16_t seconds;

            if (!strcmp_P(res->arg1, PSTR("reset"))) {
                    eeprom_write_word(EEPROM_TIME_ADDRESS, 0);
            }
            seconds = eeprom_read_word(EEPROM_TIME_ADDRESS);
            printf_P(PSTR("Running since %d mn %d\r\n"), seconds/60, seconds%60);
     */
#endif
}

prog_char str_time_monitor_arg0[] = "time_monitor";
parse_pgm_token_string_t cmd_time_monitor_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_time_monitor_result, arg0, str_time_monitor_arg0);
prog_char str_time_monitor_arg1[] = "show#reset";
parse_pgm_token_string_t cmd_time_monitor_arg1 = TOKEN_STRING_INITIALIZER(struct cmd_time_monitor_result, arg1, str_time_monitor_arg1);

prog_char help_time_monitor[] = "Show since how long we are running";
parse_pgm_inst_t cmd_time_monitor = {
    .f = cmd_time_monitor_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_time_monitor,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_time_monitor_arg0,
        (prog_void *) & cmd_time_monitor_arg1,
        NULL,
    },
};

/**********************************************************/
/* Sleep */

/* this structure is filled when cmd_sleep is parsed successfully */
struct cmd_sleep_result
{
    fixed_string_t arg0;
    uint32_t ms;
};

/* function called when cmd_sleep is parsed successfully */
static void cmd_sleep_parsed(void *parsed_result, void *data)
{
    struct cmd_sleep_result *res = parsed_result;
    time_wait_ms(res->ms);
}

prog_char str_sleep_arg0[] = "sleep";
parse_pgm_token_string_t cmd_sleep_arg0 = TOKEN_STRING_INITIALIZER(struct cmd_sleep_result, arg0, str_sleep_arg0);
parse_pgm_token_num_t cmd_sleep_ms = TOKEN_NUM_INITIALIZER(struct cmd_sleep_result, ms, UINT32);

prog_char help_sleep[] = "Sleep during some miliseconds";
parse_pgm_inst_t cmd_sleep = {
    .f = cmd_sleep_parsed, /* function to call */
    .data = NULL, /* 2nd arg of func */
    //.help_str = help_sleep,
    .tokens =
    { /* token list, NULL terminated */
        (prog_void *) & cmd_sleep_arg0,
        (prog_void *) & cmd_sleep_ms,
        NULL,
    },
};


#endif /* COMPILE_COMMANDS_MAINBOARD_OPTIONALS -------------------------------*/
