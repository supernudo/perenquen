/*
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2010)
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

#include <stdint.h>

#include <uart.h>
#include <main.h>
#include "telemetry.h"

/* Telemetry data packet */
struct tm_data tm;

/* Initialize header of telemetry packet */
void tm_data_init(void)
{
	tm.header[0] = TM_HEADER_BYTE_0;
	tm.header[1] = TM_HEADER_BYTE_1;
	tm.header[2] = TM_HEADER_BYTE_2;
	tm.header[3] = TM_HEADER_BYTE_3;
}

/* Update telemetry packet and send it thru UART */
void tm_data_send(void)
{
	void *pdata = (void*)&tm;
	uint8_t data_size = sizeof(struct tm_data);
  uint8_t i;

	/* timetag */
	tm.time_ms = time_get_us2()/1000;

	/* angle CS */
	tm.angle.consign = cs_get_consign(&mainboard.angle.cs);
	tm.angle.fconsign = cs_get_filtered_consign(&mainboard.angle.cs);
	tm.angle.error = cs_get_error(&mainboard.angle.cs);
	tm.angle.ffeedback = cs_get_filtered_feedback(&mainboard.angle.cs);
	tm.angle.out = cs_get_out(&mainboard.angle.cs);

	/* distance CS */
	tm.distance.consign = cs_get_consign(&mainboard.distance.cs);
	tm.distance.fconsign = cs_get_filtered_consign(&mainboard.distance.cs);
	tm.distance.error = cs_get_error(&mainboard.distance.cs);
	tm.distance.ffeedback = cs_get_filtered_feedback(&mainboard.distance.cs);
	tm.distance.out = cs_get_out(&mainboard.distance.cs);

	/* TODO: add more data */

	/* send data */
	for(i=0; i<data_size; i++)
		uart_send(0,*(char*)pdata++);
}
