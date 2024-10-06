/*
 *  Copyright Javier Baliñas Santos (2024)
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

#include <stdio.h>
#include <string.h>

#include <aversive.h>
#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>

#include "leds.h"

/** 
 *  HC05 setup. To be executed as main program. 
 *  Turn all LEDs and the end and never returns 
 */
void hc05_setup(void) 
{
	/* Enable interrupts */
	sei();

    /* Test/ping */
	printf("AT\n\r");
	wait_ms(100);
	printf("AT\n\r");
	wait_ms(100);

    /* Restore default */
	printf("AT+ORGL\n\r");
	wait_ms(1000);
	printf("AT\n\r");
	wait_ms(100);

    /* Set password */
	//printf("AT+PSWD=goma\n\r");
	//wait_ms(100);

    /* Set baudrate */
	//printf("AT+UART=115200,0,0\n\r");
	//printf("AT+UART=230400,0,0\n\r");
	//printf("AT+UART=460800,0,0\n\r");
	printf("AT+UART=921600,0,0\n\r");
	//printf("AT+UART=1382400,0,0\n\r");
	wait_ms(100);

    /* Notify process ends */
    /* LEDS */
	LED1_ON();
	LED2_ON();
	LED3_ON();
	LED4_ON();

    /* Block*/
	while(1);
}
