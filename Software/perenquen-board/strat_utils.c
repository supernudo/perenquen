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
 *  Copyright Javier Baliñas Santos (2018)
 *  Javier Baliñas Santos <balinas@gmail.com>
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
#include "strat_utils.h"
#include "strat.h"
#include "strat_base.h"
#include "sensor.h"


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
