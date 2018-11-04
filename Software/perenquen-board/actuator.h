/*
 *  Copyright Javier Baliñas Santos (2018)
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
 *  Javier Baliñas Santos <balinas@gmail.com>
 */

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include <stdint.h>

/* High Speed PWM module driver */
#define HSPWM_DUTY_MAX 6000
void hspwm_init(void);
void hspwm_set_pwm(void* gen_num, int16_t val) ;

/* used by cs, correct offset and save values */
void motor_pwm_set_and_save(void *pwm_gen_num, int32_t val);


#endif
