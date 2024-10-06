
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

#include <scheduler.h>
#include <oscillator.h>

#include "main.h"
#include "sensor.h"
#include "leds.h"

#ifndef HOST_VERSION

/** Board IO pin init */
void io_setup(void)
{
	/***************************************
 	*  IO portmap and config
 	*/

	/* XXX: after reset all pins are inputs */
	/* XXX: after reset all ANALOG pins are analog
	 *		  and has disabled the read operation
	 */

	/* analog inputs */
	/* set all analog pins to digital input */
	ANSELB = 0x00;
	ANSELC = 0x00;
	ANSELD = 0x00;
	ANSELE = 0x00;
	ANSELG = 0x00;

	/* leds */
	_TRISG3 = 0; /* MAIN_LED1 */
	_LATG3	= 0;
	//_TRISF6 = 0; /* MAIN_LED2 */
	//_LATF6	= 0;
	_TRISF2 = 0; /* MAIN_LED3 */
	_LATF2	= 0;
 	_TRISF3 = 0; /* MAIN_LED4 */
	_LATF3	= 0;

	/* motors */
	_TRISE3 = 0; /* PWM2H/RE3 MOTOR-L-INB */
	_LATE3	= 0;
	_TRISE2 = 0; /* PWM2L/RE2 MOTOR-L-INA */
	_LATE2	= 0;
	_TRISE1 = 0; /* PWM1H/RE1 MOTOR-R-INB */
	_LATE1	= 0;
	_TRISE0 = 0; /* PWM1L/RE0 MOTOR-R-INA */
	_LATE0	= 0;

	/* encoders */
	_QEA1R 	= 70;	/* QEA1 <- RP70(RD6) <- R_ENC_CHA */
	_TRISD6 = 1;
	_QEB1R 	= 71;	/* QEB1 <- RP71(RD7) <- R_ENC_CHB */
	_TRISD7	= 1;

	/* XXX: encoder channels swaped for equal inc/dec sign with right encoder */
	_QEA2R 	= 97;	/* QEA2 <- RP96(RF0) <- L_ENC_CHA */
	_TRISF0 = 1;
	_QEB2R 	= 96;	/* QEB2 <- RP97(RF1)  <- L_ENC_CHB */
	_TRISF1	= 1;

	/* TX wall sensors */
	_TRISD5 = 0; /* FRONT-L-TX */
	_LATD5  = 0;
	_TRISD2 = 0; /* FRONT-R-TX */
	_LATD2  = 0;
	_TRISD4 = 0; /* DIAG1-TX */
	_LATD4  = 0;
	_TRISD3 = 0; /* DIAG2-TX */
	_LATD3  = 0;

	/* RX wall sensors */
	_ANSB1 = 1;	 /* AN1/RB1 FRONT-L-RX */
	_ANSB5 = 1;	 /* AN5/RB5 FRONT-R-RX */
	_ANSB3 = 1;	 /* AN3/RB3 DIAG-L-RX */
	_ANSB4 = 1;	 /* AN4/RB4 DIAG-R-RX */
	_ANSB2 = 1;	 /* AN2/RB2 FLASH-RX */

	/* battery */
	_ANSB0 = 1;	 /* AN0/RB0 VBAT-SENS */

	/* gyro */
	_ANSE5 = 1;	 /* AN29/RE5 GYRO-VREF */
	_ANSE6 = 1;	 /* AN30/RE6 GYRO-OUTZ */

	/* user switches */
	_TRISB15 = 1; /* USER_SW1 */
	_TRISB14 = 1; /* USER_SW2 */

	/* bluetooth */
	_TRISB13 = 1;	/* BT-STATE */
	_TRISB12 = 0;	/* BT-EN */
	_LATB12  = 1;

	/* uart, U1 is for cmdline */
	_U1RXR  = 101;
	_RP100R = 0b00001;
	_TRISF4 = 0;
	_TRISF5 = 1;
}

/** ADC 1 setup, intended to sense analog sensors */
void adc1_setup(void)
{
	/* Use ADC1 for allow control sample time and conversion */

	/* Set default config */
	AD1CON1 = 0;
	AD1CON2 = 0;
	AD1CON3 = 0;
	AD1CON4 = 0;

	/* Clearing the Sample bit (SAMP) ends sampling and starts conversion (Manual mode) */
	AD1CON1bits.SSRC = 0;

	/* Channel Select bits
			1x = Converts CH0, CH1, CH2 and CH3
			01 = Converts CH0 and CH1
			00 = Converts CH0
	*/
	AD1CON2bits.CHPS = 0;

	/* Increment Rate bits
			01111 = Generates interrupt after completion of every 16th sample/conversion operation
			01110 = Generates interrupt after completion of every 15th sample/conversion operation
			•
			00001 = Generates interrupt after completion of every 2nd sample/conversion operation
			00000 = Generates interrupt after completion of every sample/conversion operation
	*/
	AD1CON2bits.SMPI = 0;

	/* ADC Conversion Clock Select bits
	11111111 = TP • (ADCS<7:0> + 1) = 256 • TCY = TAD
	•
	•
	•
	00000010 = TP • (ADCS<7:0> + 1) = 3 • TCY = TAD
	00000001 = TP • (ADCS<7:0> + 1) = 2 • TCY = TAD
	00000000 = TP • (ADCS<7:0> + 1) = 1 • TCY = TAD
	*/
	/* 
	   TCY = 1/FCY = 1/60MhZ = 16,67ns
	   TADmin@10bits = 76ns
	   ADCS = 76ns/16,67ns = 5 --> TAD = 83,335ns 
	*/
	#define ADC_TAD_ns 	(83.335)
	AD1CON3bits.ADCS = 5;

	/* Auto-Sample Time bits
			11111 = 31 TAD
			•
			00001 = 1 TAD
			00000 = 0 TAD
	*/
	//AD1CON3bits.SAMC = 2;

	/* Channel 1, 2, 3 Positive Input Select for Sample A bit
			1 = CH1 positive input is AN3, CH2 positive input is AN4, CH3 positive input is AN5
			0 = CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
	*/
	AD1CHS123bits.CH123SA = 1;

	/* Channel 0 Positive Input Select for Sample A bits */
	AD1CHS0bits.CH0SA = 0;

	/* interrupt */
	//_AD1IF = 0;
	//_AD1IE = 1;

	/* ADC module is operating */
	AD1CON1bits.ADON = 1;
}


/** Timer 1 setup, intended to use as the main timer for scheduler */
void timer1_setup(void)
{
	/* use timer 1 */
	T1CON = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	TMR1 = 0x0000;
	PR1 = SCHEDULER_UNIT * (unsigned long)((double)FCY / 1000000.0);
	T1CONbits.TON = 1;
}

/** Timer 1 interrupt */
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
    _T1IF=0;
    sei();
	scheduler_interrupt();
}

/** Timer 2 init, intended to use for ADC sensor reading */
void timer2_setup(void)
{
	#define TIMER2_PERIOD_us 100L
	T2CON = 0;
	IFS0bits.T2IF = 0;
	IEC0bits.T2IE = 1;
	TMR2 = 0x0000;
	PR2 = TIMER2_PERIOD_us * (unsigned long)((double)FCY / 1000000.0);
	T2CONbits.TON = 1;
}

/** Timer 2 interrupt */
void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
  	_T2IF=0;
	sensor_do_adc_read_sequence();
}

#endif /* !HOST_VERSION */
