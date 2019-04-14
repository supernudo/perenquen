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

#ifndef _TELEMETRY_H_
#define _TELEMETRY_H_

#include <stdint.h>

#include <main.h>
#include "telemetry.h"

/* Telemetry data of each CS */
struct tm_cs_data
{
	/* total 5 x 4 bytes = 20 bytes */
	int32_t consign;
	int32_t fconsign;
	int32_t error;
	int32_t ffeedback;
	int32_t out;
};

/* Telemetry data packet */
struct tm_data
{
	#define TM_HEADER_BYTE_0 'T'
	#define TM_HEADER_BYTE_1 'M'
	#define TM_HEADER_BYTE_2 'D'
	#define TM_HEADER_BYTE_3 'T'
	
	uint8_t header[4];						// 4 bytes
	uint32_t time_ms;							// 4 bytes
	struct tm_cs_data angle;			// 20 bytes
	struct tm_cs_data distance;		// 20 bytes
																// 48 bytes total

	// wall sensors: 4 x 2 = 8 bytes
	// gyro: 								 2 bytes
	// battery: 						 1 byte
	// encoders:		 2 x 4 = 8 bytes
	// x, y, a:			 3 x 2 = 6 bytes
	// i, j:				 2 x 1 = 2 bytes
	// slot_info:						 1 byte
	// total:							  74 bytes
};


/* Initialize header of telemetry packet */
void tm_data_init(void);

/* Update telemetry packet and send it thru UART */
void tm_data_send(void);

#endif /* _TELEMETRY_H_ */
