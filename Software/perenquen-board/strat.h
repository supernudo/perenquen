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
 *  Javier Baliñas Santos <balinas@gmail.com> and Silvia Santano
 */

#ifndef _STRAT_H_
#define _STRAT_H_

#ifndef HOST_VERSION_OA_TEST
 #include <clock_time.h>
#endif

/* compilation flavours */
//#define HOMOLOGATION

/* convert coords according to our color */
#define COLOR_Y(y)     (y)
#define COLOR_X(x)     ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (AREA_X-(x)))

#define COLOR_A_REL(a) ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (-a))
#define COLOR_A_ABS(a) ((mainboard.our_color==I2C_COLOR_YELLOW)? (a) : (180-a))

#define COLOR_SIGN(x)  ((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (-x))
#define COLOR_INVERT(x)((mainboard.our_color==I2C_COLOR_YELLOW)? (x) : (!x))

#define START_X 200
#define START_Y COLOR_Y(200)
#define START_A COLOR_A(45)

#define CENTER_X 1500
#define CENTER_Y 1000

#define SIDE_LEFT		    0
#define SIDE_RIGHT 	    1
#define SIDE_FRONT      2
#define SIDE_REAR       3
#define SIDE_ALL		    4

#define GO_FORWARD	    0
#define GO_BACKWARD	    1

/* useful traj flags */
#define TRAJ_SUCCESS(f) 			(f & (END_TRAJ|END_NEAR))
#define TRAJ_BLOCKING(f) 			(f & (END_BLOCKING))

#define TRAJ_FLAGS_STD 				(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_TIMER 		(END_TRAJ|END_BLOCKING|END_NEAR|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_NO_NEAR 			(END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR|END_TIMER)
#define TRAJ_FLAGS_NO_NEAR_NO_TIMER (END_TRAJ|END_BLOCKING|END_OBSTACLE|END_INTR)
#define TRAJ_FLAGS_SMALL_DIST 		(END_TRAJ|END_BLOCKING|END_INTR)

#define LAST_SECONDS_TIME	80

//#define CALIBRATION
#ifdef CALIBRATION

/* default acc */
#define ACC_DIST  50
#define ACC_ANGLE 50

/* default speeds */
#define SPEED_DIST_VERY_FAST 	  5000.
#define SPEED_ANGLE_VERY_FAST 	10000.
#define SPEED_DIST_FAST 		4000.
#define SPEED_ANGLE_FAST 		4000.
#define SPEED_DIST_SLOW 		2500.
#define SPEED_ANGLE_SLOW 		2500.
#define SPEED_DIST_VERY_SLOW 	1000.
#define SPEED_ANGLE_VERY_SLOW	1000.

#else

/* default acc */
#undef VERY_GOOD_CONDITIONS_OR_SPECIAL_CASES
#ifdef VERY_GOOD_CONDITIONS_OR_SPECIAL_CASES
#define ACC_DIST  70
#define ACC_ANGLE 60
#else
#define ACC_DIST  3
#define ACC_ANGLE 3
#endif

/* default speeds */
#define SPEED_DIST_VERY_FAST 	600
#define SPEED_DIST_FAST 		  600
#define SPEED_ANGLE_VERY_FAST 600
#define SPEED_ANGLE_FAST 		  600

//Do not change
#define SPEED_DIST_SLOW 		   600
#define SPEED_ANGLE_SLOW 		   600
#define SPEED_DIST_VERY_SLOW 	 600
#define SPEED_ANGLE_VERY_SLOW  600

#ifdef HOST_VERSION
#define ACC_DIST  72
#define ACC_ANGLE 72
#define SPEED_DIST_VERY_FAST 	5000
#define SPEED_DIST_FAST 		  4000
#define SPEED_ANGLE_VERY_FAST 5000
#define SPEED_ANGLE_FAST 		  4000
#endif


#endif

/*
 * Strat diagram, valid for YELLOW.
 *
 * s1: orphan stand 1
 * G1: stand group 1
 * c1: popcorn cup 1
 *
 *           popcorn                         opp popcorn
 *        G4 machines  G3                    machines
 * 2000 +-!---|----|---!+--------+--------+----|----|-----+
 *      | s1          s2|        |        |               |
 *      |             s3|        |        |               |
 *      |------·        |========|========|        ·------|
 * opp  |      |        |========|========|        |      | cinema_up
 * cinema_up   |        ·========·========·        |      |
 *      |------+--·    c1                       ·--+------|
 *      | home     |                           | opp home |
 *   y  | yellow   |                           | green    |
 *      |------+--·   s4                        ·--+------|
 * opp  |      |       · G1··s5                    |      | cinema_down
 * cinema_down |         ·                         |      |
 *      |------·       s6        c3                ·------|
 *    G2··s8 c2          ·---------------·                |
 *      | s7             | +-----------+ |                |
 *   0  +----|---|---|---+-+-----------+-+---|---|---|----+
 *      0  clap1    clap2        x             clap3      3000
 *
 */

/*************************************************************
 * Strat data structures
 ************************************************************/

/* boulding box */
struct bbox {
	int32_t x1;
	int32_t y1;
	int32_t x2;
	int32_t y2;
};

/* configuration */
struct conf {

/* depends on flags the robot
 * will do different things */
	uint8_t flags;

};



/* information about general strat stuff */
struct strat_infos {

	/* debug */
	uint8_t dump_enabled;
	uint8_t debug_step;
	uint8_t match_ends;

	/* bounding box area */
	struct bbox area_bbox;

    /* strat configuration */
	struct conf conf;
};

extern struct strat_infos strat_infos;

#ifndef HOST_VERSION_OA_TEST

/*************************************************************
 * Functions headers of strat files
 ************************************************************/

/********************************************
 * in strat.c
 *******************************************/

void strat_set_bounding_box(void);

char *get_zone_name(uint8_t zone_num);
void strat_dump_infos(const char *caller);
void strat_dump_conf(void);
void strat_reset_infos(void);

void strat_preinit(void);
void strat_init(void);
void strat_exit(void);

/* Strat main loop and events */
void strat_event(void *dummy);
uint8_t strat_main(void);


#else /* HOST_VERSION_OA_TEST */

void strat_set_bounding_box(void);

#endif /* HOST_VERSION_OA_TEST */
#endif
