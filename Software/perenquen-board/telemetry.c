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

#ifdef BT_TEST
#define TM_TEST_SIZE 30
uint8_t tm_test[TM_TEST_SIZE];
#endif

/* Initialize header of telemetry packet */
void tm_data_init(void)
{
	uint8_t i;

	tm.header[0] = TM_HEAD_BYTE_0;
	tm.header[1] = TM_HEAD_BYTE_1;
	tm.tail = TM_TAIL_BYTE_0;

#ifdef BT_TEST
	for(i=0; i<TM_TEST_SIZE; i++)
		tm_test[i] = 0x50 + i;

	tm_test[TM_TEST_SIZE-1] = '\n';
#endif
}

/* Update telemetry packet and send it thru UART */
void tm_data_send(void)
{
	void *pdata = (void*)&tm;
	uint8_t data_size = sizeof(struct tm_data);

#ifndef BT_TEST
	uint8_t i;
	uint8_t state = 0;
	static uint32_t bytes_count = 0;
#endif

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


#ifndef BT_TEST
	/* send data */
	for(i=0; i<data_size; i++)
		uart_send(0,*(char*)pdata++);
#else
	for(i=0; i<TM_TEST_SIZE; i++)
		uart_send(0,tm_test[i]);

	//switch(state) {
	//	case 0:
	//		bytes_count += TM_TEST_SIZE;
	//		//if (bytes_count < 5000)
	//		//	state = 1;
	//		break;
	//	case 1:
	//		bytes_count += 90;
	//		break;
	//	default:
	//		state = 0;
	//		break;
	//}
#endif
}
