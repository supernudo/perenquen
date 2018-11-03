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
 *  Revision : $Id: strat_utils.c,v 1.6 2009/05/27 20:04:07 zer0 Exp $
 *
 */


/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Bali�as Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  strat_utils.c,v 1.6 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <pwm_servo.h>
#include <clock_time.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include <rdline.h>
#include <parse.h>

#include "../common/i2c_commands.h"

#include "main.h"
#include "strat_utils.h"
#include "strat.h"
#include "strat_base.h"
#include "sensor.h"
#include "i2c_protocol.h"

/* return the distance between two points */
int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2)
{
	int32_t x,y;
	x = (x2-x1);
	x = x*x;
	y = (y2-y1);
	y = y*y;
	return sqrt(x+y);
}

/* return the distance to a point in the area */
int16_t distance_from_robot(int16_t x, int16_t y)
{
	return distance_between(position_get_x_s16(&mainboard.pos),
				position_get_y_s16(&mainboard.pos), x, y);
}

/* return the distance to a point in the area */
#if 0
int16_t distance_from_robot_signed(int16_t x, int16_t y)
{
	double a;
   double dis;

	abs_xy_to_rel_da(x, y, &dis, &a);

   if(a>0 && a<M_PI/2) {
      if(position_get_x_s16(&mainboard.pos)>x) dis=-dis; 
   }

   else if(a>M_PI/2 && a<M_PI){
      if(position_get_x_s16(&mainboard.pos)<x) dis=-dis;
   }

   else if(a<-M_PI/2 && a>-M_PI){
      if(position_get_x_s16(&mainboard.pos)<x) dis=-dis;
   }

   else if(a<0 && a>-M_PI/2){
      if(position_get_x_s16(&mainboard.pos)>x) dis=-dis;
   }

   return dis;
}
#endif

/** do a modulo 360 -> [-180,+180], knowing that 'a' is in [-3*180,+3*180] */  
int16_t simple_modulo_360(int16_t a)
{
	if (a < -180) {
		a += 360;
	}
	else if (a > 180) {
		a -= 360;
	}
	return a;
}


/** do a modulo 2.pi -> [-Pi,+Pi], knowing that 'a' is in [-3Pi,+3Pi] */  
/*
double simple_modulo_2pi(double a)
{
	if (a < -M_PI) {
		a += M_2PI;
	}
	else if (a > M_PI) {
		a -= M_2PI;
	}
	return a;
} */

/* return the distance to a point in the area */
int16_t angle_abs_to_rel(int16_t a_abs)
{
	return simple_modulo_360(a_abs - position_get_a_deg_s16(&mainboard.pos));
}

void rel_da_to_abs_xy(double d_rel, double a_rel_rad, 
		      double *x_abs, double *y_abs)
{
	double x = position_get_x_double(&mainboard.pos); 
	double y = position_get_y_double(&mainboard.pos);
	double a = position_get_a_rad_double(&mainboard.pos);

	*x_abs = x + d_rel*cos(a+a_rel_rad);
	*y_abs = y + d_rel*sin(a+a_rel_rad);
}

double norm(double x, double y)
{
	return sqrt(x*x + y*y);
}

void rel_xy_to_abs_xy(double x_rel, double y_rel, 
		      double *x_abs, double *y_abs)
{
	double d_rel, a_rel;
	d_rel = norm(x_rel, y_rel);
	a_rel = atan2(y_rel, x_rel);
	rel_da_to_abs_xy(d_rel, a_rel, x_abs, y_abs);
}

/* return an angle between -pi and pi */
void abs_xy_to_rel_da(double x_abs, double y_abs, 
		      double *d_rel, double *a_rel_rad)
{
	double x = position_get_x_double(&mainboard.pos); 
	double y = position_get_y_double(&mainboard.pos);
	double a = position_get_a_rad_double(&mainboard.pos);
	
	*a_rel_rad = atan2(y_abs - y, x_abs - x) - a;
	if (*a_rel_rad < -M_PI) {
		*a_rel_rad += M_2PI;
	}
	else if (*a_rel_rad > M_PI) {
		*a_rel_rad -= M_2PI;
	}
	*d_rel = norm(x_abs-x, y_abs-y);
}

void rotate(double *x, double *y, double rot)
{
	double l, a;
	
	l = norm(*x, *y);
	a = atan2(*y, *x);

	a += rot;
	*x = l * cos(a);
	*y = l * sin(a);
}

/* return true if the point is in area */
uint8_t is_in_area(int16_t x, int16_t y, int16_t margin)
{
	if (x < margin)
		return 0;
	if (x > (AREA_X - margin))
		return 0;
	if (y < margin)
		return 0;
	if (y > (AREA_Y - margin))
		return 0;
	return 1;
}

uint8_t point_is_in_area(int16_t px, int16_t py,
								 int16_t x_up, int16_t y_up,
								 int16_t x_down, int16_t y_down)
{
	if (px < x_up)
		return 0;
	if (px > x_down)
		return 0;
	if (py > y_up)
		return 0;
	if (py < y_down)
		return 0;

	return 1;
}

/* return true if the point is in area */
uint8_t robot_is_in_area(int16_t margin)
{
	return is_in_area(position_get_x_s16(&mainboard.pos),
			  position_get_y_s16(&mainboard.pos),
			  margin);
}


/* return 1 or 0 depending on which side of a line (y=cste) is the
 * robot. works in yellow or green color. */
uint8_t y_is_more_than(int16_t y)
{
	int16_t posy;
	
	posy = position_get_y_s16(&mainboard.pos);
	if (posy > y)
		return 1;
	else
		return 0;

}



/* return 1 or 0 depending on which side of a line (x=cste) is the
 * robot. works in yellow or green color. */

uint8_t x_is_more_than(int16_t x)
{
	int16_t posx;
	
	posx = position_get_x_s16(&mainboard.pos);
	if (mainboard.our_color == I2C_COLOR_YELLOW) {
		if (posx > x)
			return 1;
		else
			return 0;
	}
	else {
		if (posx < (AREA_X-x))
			return 1;
		else
			return 0;
	}
}

/* return 1 if x > x_opp or XXX opponent not there */
uint8_t opp1_x_is_more_than(int16_t x)
{
	int16_t x_opp, y_opp;
	
	if(get_opponent1_xy(&x_opp, &y_opp) == -1)
		return 1;

	if (mainboard.our_color == I2C_COLOR_YELLOW) {
		if (x_opp > x)
			return 1;
		else
			return 0;
	}
	else {
		if (x_opp < (AREA_X-x))
			return 1;
		else
			return 0;
	}
}

/* return 1 if x > x_opp or opponent not there */
uint8_t opp2_x_is_more_than(int16_t x)
{
#ifdef TWO_OPPONENTS
	int16_t x_opp2, y_opp2;
	
	if(get_opponent2_xy(&x_opp2, &y_opp2) == -1)
		return 1;

	if (mainboard.our_color == I2C_COLOR_YELLOW) {
		if (x_opp2 > x)
			return 1;
		else
			return 0;
	}
	else {
		if (x_opp2 < (AREA_X-x))
			return 1;
		else
			return 0;
	}
#endif
   return 0;
}

/* return 1 if x > x_opp or opponent not there */
uint8_t robot_2nd_x_is_more_than(int16_t x)
{
#ifdef ROBOT_2ND
	int16_t x_robot_2nd, y_robot_2nd;
	
	if(get_robot_2nd_xy(&x_robot_2nd, &y_robot_2nd) == -1)
		return 1;

	if (mainboard.our_color == I2C_COLOR_YELLOW) {
		if (x_robot_2nd > x)
			return 1;
		else
			return 0;
	}
	else {
		if (x_robot_2nd < (AREA_X-x))
			return 1;
		else
			return 0;
	}
#endif
   return 0;
}

/* return 1 if y > y_opp or opponent not there */
uint8_t opp1_y_is_more_than(int16_t y)
{
	int16_t x_opp, y_opp;
	
	if(get_opponent1_xy(&x_opp, &y_opp) == -1)
		return 1;
	if (y_opp > y)
		return 1;
	else
		return 0;
}
/* return 1 if y > y_opp or opponent not there */
uint8_t opp2_y_is_more_than(int16_t y)
{
#ifdef TWO_OPPONENTS
	int16_t x_opp2, y_opp2;
	
	if(get_opponent2_xy(&x_opp2, &y_opp2) == -1)
		return 1;

	if (y_opp2 > y)
		return 1;
	else
		return 0;
#endif
   return 0;

}
/* return 1 if y > y_opp or opponent not there */
uint8_t robot_2nd_y_is_more_than(int16_t y)
{
#ifdef ROBOT_2ND
	int16_t x_robot_2nd, y_robot_2nd;
	
	if(get_robot_2nd_xy(&x_robot_2nd, &y_robot_2nd) == -1)
		return 1;

	if (y_robot_2nd > y)
		return 1;
	else
		return 0;

#endif
   return 0;
}

int16_t sin_table[] = {
	0,
	3211,
	6392,
	9512,
	12539,
	15446,
	18204,
	20787,
	23170,
	25330,
	27245,
	28898,
	30273,
	31357,
	32138,
	32610,
	32767,
};

int16_t fast_sin(int16_t deg)
{
	deg %= 360;
	
	if (deg < 0)
		deg += 360;

	if (deg < 90) 
		return sin_table[(deg*16)/90];
	else if (deg < 180) 
		return sin_table[((180-deg)*16)/90];
	else if (deg < 270) 
		return -sin_table[((deg-180)*16)/90];
	else
		return -sin_table[((360-deg)*16)/90];
}

int16_t fast_cos(int16_t deg)
{
	return fast_sin(90+deg);
}


/* get the color of our robot */
uint8_t get_color(void)
{
	return mainboard.our_color;
}

/* get the color of the opponent robot */
uint8_t get_opponent_color(void)
{
	if (mainboard.our_color == I2C_COLOR_GREEN)
		return I2C_COLOR_YELLOW;
	else
		return I2C_COLOR_GREEN;
}


#ifdef IM_SECONDARY_ROBOT
int8_t get_best_opponent1_xyda (int16_t *x, int16_t *y, int16_t *d, int16_t *a)
{
	uint8_t flags;

	if (beaconboard.opponent1_x == I2C_OPPONENT_NOT_THERE &&
		beaconboard.opponent2_x == I2C_OPPONENT_NOT_THERE) 
	{
		IRQ_LOCK(flags);
		if (x != NULL)
			*x = robot_2nd.opponent1_x;
		if (y != NULL)
			*y = robot_2nd.opponent1_y;
		if (d != NULL)
			*d = robot_2nd.opponent1_d;
		if (a != NULL)
			*a = robot_2nd.opponent1_a;
		IRQ_UNLOCK(flags);	
	} 
	else {
		IRQ_LOCK(flags);
		if (x != NULL)
			*x = beaconboard.opponent1_x;
		if (y != NULL)
			*y = beaconboard.opponent1_y;
		if (d != NULL)
			*d = beaconboard.opponent1_d;
		if (a != NULL)
			*a = beaconboard.opponent1_a;
		IRQ_UNLOCK(flags);	
	}
	return 0;
}

int8_t get_best_opponent2_xyda (int16_t *x, int16_t *y, int16_t *d, int16_t *a)
{
	uint8_t flags;

	if (beaconboard.opponent1_x == I2C_OPPONENT_NOT_THERE &&
		beaconboard.opponent2_x == I2C_OPPONENT_NOT_THERE) 
	{
		IRQ_LOCK(flags);
		if (x != NULL)
			*x = robot_2nd.opponent2_x;
		if (y != NULL)
			*y = robot_2nd.opponent2_y;
		if (d != NULL)
			*d = robot_2nd.opponent2_d;
		if (a != NULL)
			*a = robot_2nd.opponent2_a;
		IRQ_UNLOCK(flags);	
	} 
	else {
		IRQ_LOCK(flags);
		if (x != NULL)
			*x = beaconboard.opponent2_x;
		if (y != NULL)
			*y = beaconboard.opponent2_y;
		if (d != NULL)
			*d = beaconboard.opponent2_d;
		if (a != NULL)
			*a = beaconboard.opponent2_a;
		IRQ_UNLOCK(flags);	
	}
	return 0;
}

#endif

/* get the xy pos of a robot */
int8_t get_opponent1_xy(int16_t *x, int16_t *y)
{
#ifndef IM_SECONDARY_ROBOT
	uint8_t flags;
	IRQ_LOCK(flags);
	*x = beaconboard.opponent1_x;
	*y = beaconboard.opponent1_y;
	IRQ_UNLOCK(flags);
#else
	get_best_opponent1_xyda (x, y, NULL, NULL);
#endif

	if (*x == I2C_OPPONENT_NOT_THERE)
		return -1;
	return 0;
}
int8_t get_opponent2_xy(int16_t *x, int16_t *y)
{
#ifdef TWO_OPPONENTS

#ifndef IM_SECONDARY_ROBOT
	uint8_t flags;
	IRQ_LOCK(flags);
	*x = beaconboard.opponent2_x;
	*y = beaconboard.opponent2_y;
    IRQ_UNLOCK(flags);
#else
	get_best_opponent2_xyda (x, y, NULL, NULL);
#endif

	if (*x == I2C_OPPONENT_NOT_THERE)
		return -1;
#endif
	return 0;
}
int8_t get_robot_2nd_xy(int16_t *x, int16_t *y)
{
#ifdef ROBOT_2ND
	uint8_t flags;

	IRQ_LOCK(flags);
	*x = robot_2nd.x;
	*y = robot_2nd.y;
	IRQ_UNLOCK(flags);
	if (*x == I2C_OPPONENT_NOT_THERE)
		return -1;
#endif
	return 0;
}

/* get the da pos of a robot */
int8_t get_opponent1_da(int16_t *d, int16_t *a)
{
	int16_t x_tmp;

#ifndef IM_SECONDARY_ROBOT
	uint8_t flags;
	IRQ_LOCK(flags);
	x_tmp = beaconboard.opponent1_x;
	*d = beaconboard.opponent1_d;
	*a = beaconboard.opponent1_a;
	IRQ_UNLOCK(flags);
#else
	get_best_opponent1_xyda (&x_tmp, NULL, d, a);
#endif

	if (x_tmp == I2C_OPPONENT_NOT_THERE)
		return -1;
	return 0;
}
int8_t get_opponent2_da(int16_t *d, int16_t *a)
{
#ifdef TWO_OPPONENTS
	int16_t x_tmp;

#ifndef IM_SECONDARY_ROBOT
	uint8_t flags;
	IRQ_LOCK(flags);
	x_tmp = beaconboard.opponent2_x;
	*d = beaconboard.opponent2_d;
	*a = beaconboard.opponent2_a;
    IRQ_UNLOCK(flags);
#else
	//get_best_opponent1_xyda (&x_tmp, NULL, d, a);  
	//XXX
	get_best_opponent2_xyda (&x_tmp, NULL, d, a);
#endif

	if (x_tmp == I2C_OPPONENT_NOT_THERE)
		return -1;
#endif
	return 0;
}
int8_t get_robot_2nd_da(int16_t *d, int16_t *a)
{
#ifdef ROBOT_2ND
	uint8_t flags;
	int16_t x_tmp;

	IRQ_LOCK(flags);
	x_tmp = robot_2nd.x;
	*d = robot_2nd.d;
	*a = robot_2nd.a;
	IRQ_UNLOCK(flags);
	if (x_tmp == I2C_OPPONENT_NOT_THERE)
		return -1;
#endif
	return 0;
}

/* get the absolute a of the robot 2nd */
int8_t get_robot_2nd_a_abs(int16_t *a)
{
#ifdef ROBOT_2ND
	uint8_t flags;
	int16_t x_tmp;

	IRQ_LOCK(flags);
	x_tmp = robot_2nd.x;
	*a = robot_2nd.a_abs;
	IRQ_UNLOCK(flags);
	if (x_tmp == I2C_OPPONENT_NOT_THERE)
		return -1;
#endif
	return 0;
}

/* get the xyda pos of a robot */
int8_t get_opponent1_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a)
{
#ifndef IM_SECONDARY_ROBOT
	uint8_t flags;
	IRQ_LOCK(flags);
	*x = beaconboard.opponent1_x;
	*y = beaconboard.opponent1_y;
	*d = beaconboard.opponent1_d;
	*a = beaconboard.opponent1_a;
	IRQ_UNLOCK(flags);
#else
	get_best_opponent1_xyda (x, y, d, a);
#endif

	if (*x == I2C_OPPONENT_NOT_THERE)
		return -1;
	return 0;
}
int8_t get_opponent2_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a)
{
#ifdef TWO_OPPONENTS
#ifndef IM_SECONDARY_ROBOT
	uint8_t flags;
	IRQ_LOCK(flags);
	*x = beaconboard.opponent2_x;
	*y = beaconboard.opponent2_y;
	*d = beaconboard.opponent2_d;
	*a = beaconboard.opponent2_a;
	IRQ_UNLOCK(flags);
#else
	//get_best_opponent1_xyda (x, y, d, a);
	//XXX
	get_best_opponent2_xyda (x, y, d, a);
#endif

	if (*x == I2C_OPPONENT_NOT_THERE)
		return -1;
#endif
	return 0;
}
int8_t get_robot_2nd_xyda(int16_t *x, int16_t *y, int16_t *d, int16_t *a)
{
#ifdef ROBOT_2ND
	uint8_t flags;

	IRQ_LOCK(flags);
	*x = robot_2nd.x;
	*y = robot_2nd.y;
	*d = robot_2nd.d;
	*a = robot_2nd.a;
	IRQ_UNLOCK(flags);
	if (*x == I2C_OPPONENT_NOT_THERE)
		return -1;
#endif
	return 0;
}

/* check if a robot is behind */
uint8_t opponent1_is_behind(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_opponent1_da(&opp_d, &opp_a);

	if(opp_there == -1)
		return 0;

	if ((opp_a < 215 && opp_a > 145) && opp_d < 500)
		return 1;

	return 0;
}
uint8_t opponent2_is_behind(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_opponent2_da(&opp_d, &opp_a);

	if(opp_there == -1)
		return 0;

	if ((opp_a < 215 && opp_a > 145) && opp_d < 500)
		return 1;

	return 0;
}
uint8_t robot_2nd_is_behind(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_robot_2nd_da(&opp_d, &opp_a);

	if(opp_there == -1)
		return 0;

	if ((opp_a < 215 && opp_a > 145) && opp_d < 500)
		return 1;

	return 0;
}


/* check if a robot is in front */
uint8_t opponent1_is_infront(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_opponent1_da(&opp_d, &opp_a);
	
	if(opp_there == -1)
		return 0;

	if ((opp_a > 325 || opp_a < 35) && opp_d < 500)
		return 1;

	return 0;
}
uint8_t opponent2_is_infront(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_opponent2_da(&opp_d, &opp_a);
	
	if(opp_there == -1)
		return 0;

	if ((opp_a > 325 || opp_a < 35) && opp_d < 500)
		return 1;

	return 0;
}
uint8_t robot_2nd_is_infront(void)
{
	int8_t opp_there;
	int16_t opp_d, opp_a;

	opp_there = get_robot_2nd_da(&opp_d, &opp_a);
	
	if(opp_there == -1)
		return 0;

	if ((opp_a > 325 || opp_a < 35) && opp_d < 500)
		return 1;

	return 0;
}

/* returns 1 if any robot is near */
uint8_t robots_are_near(void)
{
	int8_t opp_there, opp2_there=-1, r2nd_there=-1;
	int16_t opp_d, opp_a;

	opp_there = get_opponent1_da(&opp_d, &opp_a);
#ifdef TWO_OPPONENTS
	int16_t opp2_d, opp2_a;
	opp2_there = get_opponent2_da(&opp2_d, &opp2_a);
#endif
#ifdef ROBOT_2ND
	//int16_t robot_2nd_d, robot_2nd_a;
	//r2nd_there = get_robot_2nd_da(&robot_2nd_d, &robot_2nd_a);
#endif
	
	if((opp_there == -1) && (opp2_there == -1) && (r2nd_there == -1))
		return 0;

	if (opp_d < 500)
		return 1;

#ifdef TWO_OPPONENTS
	if (opp2_d < 500)
		return 1;
#endif

#ifdef ROBOT_2ND
	//if (robot_2nd_d < 500)
	//	return 1;
#endif

	return 0;
}

/* check if a robot is in area */
/* return 1 if there are opponents in area, XXX pass coordinates with COLOR macro */
uint8_t opponents_are_in_area(int16_t x_up, int16_t y_up,int16_t x_down, int16_t y_down)
{
	int8_t opp1_there, opp2_there;
	int16_t opp_x, opp_y;
	int16_t opp2_x, opp2_y;

	/* get robot coordenates */
	opp1_there = get_opponent1_xy(&opp_x, &opp_y);
	opp2_there = get_opponent2_xy(&opp2_x, &opp2_y);

	//DEBUG (E_USER_STRAT, "opp1 x=%d, y=%d / opp2 x=%d, y=%d", opp_x, opp_y, opp2_x, opp2_y);

	
	/* return if no robots */
	if(opp1_there == -1 && opp2_there == -1)
		return 0;

	/* Opponent 1 */
	if (mainboard.our_color == I2C_COLOR_GREEN) {
		if ((opp_x > x_up && opp_x < x_down)
			&& (opp_y < y_up && opp_y > y_down) )
			return 1;
	}
	else {
		if ((opp_x < x_up && opp_x > x_down)
			 && (opp_y < y_up && opp_y > y_down) )
			return 1;
	}
	
	/* Opponent 2 */
	if (mainboard.our_color == I2C_COLOR_GREEN) {
		if ((opp2_x > x_up && opp2_x < x_down)
			&& (opp2_y < y_up && opp2_y > y_down) )
			return 1;
	}
	else {
		if ((opp2_x < x_up && opp2_x > x_down)
			 && (opp2_y < y_up && opp2_y > y_down) )
			return 1;
	}

	return 0;
}
uint8_t opponent1_is_in_area(int16_t x_up, int16_t y_up, int16_t x_down, int16_t y_down)
{
	int8_t opp_there;
	int16_t opp_x, opp_y;


	/* get robot coordenates */
	opp_there = get_opponent1_xy(&opp_x, &opp_y);

	/* return if no robots */
	if(opp_there == -1)
		return 0;


	/* Opponent 1 */
	if (mainboard.our_color == I2C_COLOR_YELLOW) {
		if ((opp_x > x_up && opp_x < x_down)
			&& (opp_y < y_up && opp_y > y_down) )
			return 1;
	}
	else {
		if ((opp_x < x_up && opp_x > x_down)
			 && (opp_y < y_up && opp_y > y_down) )
			return 1;
	}

	return 0;
}
uint8_t opponent2_is_in_area(int16_t x_up, int16_t y_up, int16_t x_down, int16_t y_down)
{
	int8_t opp2_there;
	int16_t opp2_x, opp2_y;

	/* get robot coordenates */
	opp2_there = get_opponent2_xy(&opp2_x, &opp2_y);

	/* return if no robots */
	if(opp2_there == -1)
		return 0;


	/* Opponent 2 */
	if (mainboard.our_color == I2C_COLOR_YELLOW) {
		if ((opp2_x > x_up && opp2_x < x_down)
			&& (opp2_y < y_up && opp2_y > y_down) )
			return 1;
	}
	else {
		if ((opp2_x < x_up && opp2_x > x_down)
			 && (opp2_y < y_up && opp2_y > y_down) )
			return 1;
	}

   return 1;
}



