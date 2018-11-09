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
 *  Revision : $Id: strat_utils.h,v 1.4 2009/05/27 20:04:07 zer0 Exp $
 *
 */

/*
 *  Copyright Javier Baliñas Santos (2018)
 *  Javier Baliñas Santos <balinas@gmail.com>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  strat_utils.h,v 1.4 2009/05/27 20:04:07 zer0 Exp.
 */

#ifndef _STRAT_UTILS_H_
#define _STRAT_UTILS_H_

#define M_2PI (2*M_PI)

#define DEG(x) ((x) * (180.0 / M_PI))
#define RAD(x) ((x) * (M_PI / 180.0))

struct xy_point {
	int16_t x;
	int16_t y;
};

/* wait traj end flag or cond. return 0 if cond become true, else
 * return the traj flag */
#define WAIT_COND_OR_TRAJ_END(cond, mask)				\
	({								\
		uint8_t __err = 0;					\
		while ( (! (cond)) && (__err == 0)) {			\
			__err = test_traj_end(mask);	\
		}							\
		__err;							\
	})								\

/* return the distance between two points */
int16_t distance_between(int16_t x1, int16_t y1, int16_t x2, int16_t y2);

/* return the distance to a point in the area */
int16_t distance_from_robot(int16_t x, int16_t y);

/* do a modulo 360 -> [-180,+180], knowing that 'a' is in [-3*180,+3*180] */
int16_t simple_modulo_360(int16_t a);

/* return the distance to a point in the area */
int16_t angle_abs_to_rel(int16_t a_abs);

void rel_da_to_abs_xy(double d_rel, double a_rel_rad,
		      double *x_abs, double *y_abs);

double norm(double x, double y);

void rel_xy_to_abs_xy(double x_rel, double y_rel,
		      double *x_abs, double *y_abs);

/* return an angle between -pi and pi */
void abs_xy_to_rel_da(double x_abs, double y_abs,
		      double *d_rel, double *a_rel_rad);

void rotate(double *x, double *y, double rot);

/* return true if the point is in area */
uint8_t is_in_area(int16_t x, int16_t y, int16_t margin);

uint8_t point_is_in_area(int16_t px, int16_t py,
								 int16_t x_up, int16_t y_up,
								 int16_t x_down, int16_t y_down);

/* return true if the point is in area */
uint8_t robot_is_in_area(int16_t margin);


/* return 1 or 0 depending on which side of a line (y=cste) is the
 * robot. works in yellow or green color. */
uint8_t y_is_more_than(int16_t y);


/* return 1 or 0 depending on which side of a line (x=cste) is the
 * robot. works in yellow or green color. */
uint8_t x_is_more_than(int16_t x);

int16_t fast_sin(int16_t deg);
int16_t fast_cos(int16_t deg);

#endif
