/*
 *  Copyright Javier Baliñas Santos (2018)
 *  Javier Baliñas Santos <balinas@gmail.com>
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
 */

/* synchronize with sensor.c */
#define S_ADC_FRONT_LEFT	  		0
#define S_ADC_FRONT_LEFT_OFF		1
#define S_ADC_FRONT_RIGHT		   	2
#define S_ADC_FRONT_RIGHT_OFF   3
#define S_ADC_DIAG_LEFT		     	  4
#define S_ADC_DIAG_LEFT_OFF        5
#define S_ADC_DIAG_RIGHT    		    6
#define S_ADC_DIAG_RIGHT_OFF        7
#define S_ADC_BATTERY       		8
#define S_ADC_GYRO          		9
#define S_ADC_GYRO_REF      		10
#define S_ADC_FLASH         		11
#define S_ADC_MAX   						12

void sensor_init(void);
void sensor_adc_do_read (uint8_t num);
uint16_t sensor_adc_get_value(uint8_t num);
uint16_t sensor_adc_get_value_mv(uint8_t num);
