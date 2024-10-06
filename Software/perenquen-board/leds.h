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

#ifndef _LEDS_H_
#define _LEDS_H_

#include <aversive.h>

/* XXX: LED 1 was disassembled during telemetry tests */
/* XXX: I think LED 2 is broken. It's disabled in code */

/* LEDs manage */
#ifndef HOST_VERSION

/* generic led toggle macro */
#define LED_TOGGLE(port, bit) do {		\
		if (port & _BV(bit))		    \
			port &= ~_BV(bit);	        \
		else				            \
			port |= _BV(bit);	        \
	} while(0)

#define LED1_ON() 		sbi(LATG, 3)
#define LED1_OFF() 		cbi(LATG, 3)
#define LED1_TOGGLE() 	LED_TOGGLE(LATG, 3)

/* FIXME: LED 2 support */
#define LED2_ON()		do {} while(0)
#define LED2_OFF()		do {} while(0)
#define LED2_TOGGLE()	do {} while(0)
//#define LED2_ON() 		sbi(LATF, 6)
//#define LED2_OFF() 		cbi(LATF, 6)
//#define LED2_TOGGLE() LED_TOGGLE(LATF, 6)

#define LED3_ON() 		sbi(LATF, 2)
#define LED3_OFF() 		cbi(LATF, 2)
#define LED3_TOGGLE()   LED_TOGGLE(LATF, 2)

#define LED4_ON() 		sbi(LATF, 3)
#define LED4_OFF() 		cbi(LATF, 3)
#define LED4_TOGGLE()   LED_TOGGLE(LATF, 3)

#else

#define LED1_ON()
#define LED1_OFF()
#define LED1_TOGGLE()

#define LED2_ON()
#define LED2_OFF()
#define LED2_TOGGLE()

#define LED3_ON()
#define LED3_OFF()
#define LED3_TOGGLE()

#define LED4_ON()
#define LED4_OFF()
#define LED4_TOGGLE()

#define BRAKE_DDR()
#define BRAKE_ON()
#define BRAKE_OFF()

#endif /* !HOST_VERSION */

#endif /*_LEDS_H_ */
