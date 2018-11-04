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
 *  Javier Baliñas Santos <javier@arc-robots.org>
 */

#ifndef HOST_VERSION

#ifndef _CONFIGURATION_BITS_CONFIG_H_
#define _CONFIGURATION_BITS_CONFIG_H_

#if defined(__dsPIC30F__)
#include <p30fxxxx.h>
#elif defined(__dsPIC33F__)
#include <p33Fxxxx.h>
#elif defined(__dsPIC33E__)
#include <p33Exxxx.h>
#elif defined(__PIC24H__)
#include <p24Hxxxx.h>
#elif defined(__PIC24F__)
#include <p24Fxxxx.h>
#endif

#if defined(__dsPIC33F__)

/* FBS: Boot Code Segment Configuration Register */
//_FBS(FBS_CONFIG);

/* FSS: Secure Code Segment Configuration Register */
//_FSS(FSS_CONFIG)

/* FGS: General Code Segment Configuration Register */
//_FGS(FGS_CONFIG);

/* FOSCSEL: Oscillator Source Selection Register */
// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC)

/* FOSC: Oscillator Configuration Register */
// Enable Clock Switching and Configure Posc in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT)

/* FWDT: Watchdog Timer (WDT) Configuration Register */
// Disable Watchdog Timer
_FWDT(FWDTEN_OFF);


/* FPOR: POR Configuration Register */
//_FPOR();

/* FICD: In-Circuit Debugger Configuration Register */
//_FICD();

/* Unit ID Field */
//_FUID1();
//_FUID2();
//_FUID3();

#elif defined(__dsPIC33E__)

/* Configuration bit macros supported by C30 compiler
**
**  - Based on dsPIC33EPXXX(GP/MC/MU)806/810/814 datasheet
**  - See [C30_compiler_folder]/support/dsPIC33E/.h for other parts support
*/

/* Register FGS (0xf80004)                               */
/*
** Only one invocation of FGS should appear in a project,
** at the top of a C source file (outside of any function).
**
** The following constants can be used to set FGS.
** Multiple options may be combined, as shown:
**
** _FGS( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**     GWRP_ON              General Segment is write protected
**     GWRP_OFF             General Segment may be written
**
**   General Segment Code-Protect bit:
**     GSS_ON               Standard Security Code protection Enabled
**     GSS_OFF              General Segment Code protect is disabled
**
**   General Segment Key bits:
**     GSSK_OFF             General Segment Write Protection and Code Protection is Disabled
**     GSSK_ON              General Segment Write Protection or Code Protection is Enabled
**
*/
_FGS(GWRP_OFF & GSS_OFF & GSSK_OFF)


/* Register FOSCSEL (0xf80006)                               */
/*
** Only one invocation of FOSCSEL should appear in a project,
** at the top of a C source file (outside of any function).
**
** The following constants can be used to set FOSCSEL.
** Multiple options may be combined, as shown:
**
** _FOSCSEL( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   Oscillator Source Selection bits:
**     FNOSC_FRC            Internal Fast RC (FRC)
**     FNOSC_FRCPLL         Internal Fast RC with PLL (FRCPLL)
**     FNOSC_PRI            Primary Oscillator (XT, HS, EC)
**     FNOSC_PRIPLL         Primary Oscillator (XT, HS, EC) with PLL
**     FNOSC_SOSC           Secondary Oscillator (SOSC)
**     FNOSC_LPRC           Low-Power RC Oscillator (LPRC)
**     FNOSC_FRCDIV16       Internal Fast RC (FRC) Oscillator with divide-by-16
**     FNOSC_FRCDIVN        Internal Fast RC (FRC) Oscillator with postscaler
**
**   Two-speed Oscillator Start-up Enable bit:
**     IESO_OFF             Start up with user-selected oscillator source
**     IESO_ON              Start up device with FRC, then switch to user-selected oscillator source
**
*/
_FOSCSEL(FNOSC_PRIPLL & IESO_OFF)

/* Register FOSC (0xf80008)                               */
/*
** Only one invocation of FOSC should appear in a project,
** at the top of a C source file (outside of any function).
**
** The following constants can be used to set FOSC.
** Multiple options may be combined, as shown:
**
** _FOSC( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   Primary Oscillator Mode Select Bit:
**     POSCMD_EC            EC (External Clock) Mode
**     POSCMD_XT            XT Crystal Oscillator Mode
**     POSCMD_HS            HS Crystal Oscillator Mode
**     POSCMD_NONE          Primary Oscillator disabled
**
**   OSC2 Pin Function Bit:
**     OSCIOFNC_ON          OSC2 is general purpose digital I/O pin
**     OSCIOFNC_OFF         OSC2 is clock output
**
**   Peripheral pin select configuration:
**     IOL1WAY_OFF          Allow multiple reconfigurations
**     IOL1WAY_ON           Allow only one reconfiguration
**
**   Clock Switching Mode bits:
**     FCKSM_CSECME         Both Clock switching and Fail-safe Clock Monitor are enabled
**     FCKSM_CSECMD         Clock switching is enabled,Fail-safe Clock Monitor is disabled
**     FCKSM_CSDCMD         Both Clock switching and Fail-safe Clock Monitor are disabled
**
*/
_FOSC(POSCMD_HS & OSCIOFNC_ON & IOL1WAY_OFF & FCKSM_CSDCMD)

/* Register FWDT (0xf8000a)                               */
/*
** Only one invocation of FWDT should appear in a project,
** at the top of a C source file (outside of any function).
**
** The following constants can be used to set FWDT.
** Multiple options may be combined, as shown:
**
** _FWDT( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   Watchdog Timer Postscaler bits:
**     WDTPOST_PS1          1:1
**     WDTPOST_PS2          1:2
**     WDTPOST_PS4          1:4
**     WDTPOST_PS8          1:8
**     WDTPOST_PS16         1:16
**     WDTPOST_PS32         1:32
**     WDTPOST_PS64         1:64
**     WDTPOST_PS128        1:128
**     WDTPOST_PS256        1:256
**     WDTPOST_PS512        1:512
**     WDTPOST_PS1024       1:1,024
**     WDTPOST_PS2048       1:2,048
**     WDTPOST_PS4096       1:4,096
**     WDTPOST_PS8192       1:8,192
**     WDTPOST_PS16384      1:16,384
**     WDTPOST_PS32768      1:32,768
**
**   Watchdog Timer Prescaler bit:
**     WDTPRE_PR32          1:32
**     WDTPRE_PR128         1:128
**
**   PLL Lock Wait Enable bit:
**     PLLKEN_OFF           Clock switch will not wait for the PLL lock signal.
**     PLLKEN_ON            Clock switch to PLL source will wait until the PLL lock signal is valid.
**
**   Watchdog Timer Window Enable bit:
**     WINDIS_ON            Watchdog Timer in Window mode
**     WINDIS_OFF           Watchdog Timer in Non-Window mode
**
**   Watchdog Timer Enable bit:
**     FWDTEN_OFF           Watchdog timer enabled/disabled by user software
**     FWDTEN_ON            Watchdog timer always enabled
**
*/
_FWDT(WDTPOST_PS32768 & WDTPRE_PR128 & PLLKEN_ON & WINDIS_OFF & FWDTEN_OFF)

/* Register FPOR (0xf8000c)                               */
/*
** Only one invocation of FPOR should appear in a project,
** at the top of a C source file (outside of any function).
**
** The following constants can be used to set FPOR.
** Multiple options may be combined, as shown:
**
** _FPOR( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   Power-on Reset Timer Value Select bits:
**     FPWRT_PWR1           Disabled
**     FPWRT_PWR2           2ms
**     FPWRT_PWR4           4ms
**     FPWRT_PWR8           8ms
**     FPWRT_PWR16          16ms
**     FPWRT_PWR32          32ms
**     FPWRT_PWR64          64ms
**     FPWRT_PWR128         128ms
**
**   Brown-out Reset (BOR) Detection Enable bit:
**     BOREN_OFF            BOR is disabled
**     BOREN_ON             BOR is enabled
**
**   Alternate I2C pins for I2C1:
**     ALTI2C1_ON           ASDA1/ASCK1 pins are selected as the I/O pins for I2C1
**     ALTI2C1_OFF          SDA1/SCK1 pins are selected as the I/O pins for I2C1
**
**   Alternate I2C pins for I2C2:
**     ALTI2C2_ON           I2C2 mapped to ASDA2/ASCL2 pins
**     ALTI2C2_OFF          I2C2 mapped to SDA2/SCL2 pins
**
*/
#ifdef ALTI2C2_OFF
_FPOR(FPWRT_PWR128 & BOREN_OFF & ALTI2C1_OFF & ALTI2C2_OFF)
#else
_FPOR(FPWRT_PWR128 & BOREN_OFF & ALTI2C1_OFF)
#endif
        
/* Register FICD (0xf8000e)                               */
/*
** Only one invocation of FICD should appear in a project,
** at the top of a C source file (outside of any function).
**
** The following constants can be used to set FICD.
** Multiple options may be combined, as shown:
**
** _FICD( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   ICD Communication Channel Select bits:
**     ICS_NONE             Reserved, do not use
**     ICS_PGD3             Communicate on PGEC3 and PGED3
**     ICS_PGD2             Communicate on PGEC2 and PGED2
**     ICS_PGD1             Communicate on PGEC1 and PGED1
**
**   Reset Target Vector Select bit:
**     RSTPRI_AF            Device will obtain reset instruction from Aux flash
**     RSTPRI_PF            Device will obtain reset instruction from Primary flash
**
**   JTAG Enable Bit:
**     JTAGEN_OFF           JTAG is disabled
**     JTAGEN_ON            JTAG is enabled
**
*/
_FICD(ICS_PGD1 & RSTPRI_PF & JTAGEN_OFF)

/* Register FAS (0xf80010)                               */
/*
** Only one invocation of FAS should appear in a project,
** at the top of a C source file (outside of any function).
**
** The following constants can be used to set FAS.
** Multiple options may be combined, as shown:
**
** _FAS( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   Auxiliary Segment Write-protect bit:
**     AWRP_ON              Aux Flash is write protected
**     AWRP_OFF             Aux Flash may be written
**
**   Auxiliary Segment Code-protect bit:
**     APL_ON               Aux Flash Code protect is enabled
**     APL_OFF              Aux Flash Code protect is disabled
**
**   Auxiliary Segment Key bits:
**     APLK_OFF             Aux Flash Write Protection and Code Protection is Disabled
**     APLK_ON              Aux Flash Write Protection or Code Protection is Enabled
**
*/
_FAS(AWRP_OFF & APL_OFF & APLK_OFF)

/* Register FUID0 (0xf80012)                               */
/*
** Only one invocation of FUID0 should appear in a project,
** at the top of a C source file (outside of any function).
**
** The following constants can be used to set FUID0.
** Multiple options may be combined, as shown:
**
** _FUID0( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   :
**
*/


#elif defined(__PIC24F__)

/* The following constants can be used to set CONFIG3.
** Multiple options may be combined, as shown:
**
** _CONFIG3( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   Write Protection Location:
**     WPEND_WPSTARTMEM     Write Protect from page 0 to WPFP
**     WPEND_WPENDMEM       Write Protect from WPFP to the last page of memory
**
**   Write Protect Configuration Page:
**     WPCFG_WPCFGEN        Enabled
**     WPCFG_WPCFGDIS       Disabled
**
**   Write Protection Disable:
**     WPDIS_WPEN           Enabled
**     WPDIS_WPDIS          Disabled
**
**   Write Protection Flash Page:
**     WPFP_WPFP0           0
**     WPFP_WPFP1           1
**     WPFP_WPFP2           2
**     ...
**     WPFP_WPFP510         510
**     WPFP_WPFP511         511
**
*/

//_CONFIG3( OPT1_ON & OPT2_OFF & OPT3_PLL )

/* The following constants can be used to set CONFIG2.
** Multiple options may be combined, as shown:
**
** _CONFIG2( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   Two Speed Start-up:
**     IESO_OFF             Disabled
**     IESO_ON              Enabled
**
**   Oscillator Selection:
**     FNOSC_FRC            Fast RC oscillator
**     FNOSC_FRCPLL         Fast RC oscillator w/ divide and PLL
**     FNOSC_PRI            Primary oscillator (XT, HS, EC)
**     FNOSC_PRIPLL         Primary oscillator (XT, HS, EC) w/ PLL
**     FNOSC_SOSC           Secondary oscillator
**     FNOSC_LPRC           Low power RC oscillator
**     FNOSC_FRCDIV         Fast RC oscillator with divide
**
**   Clock switching and clock monitor:
**     FCKSM_CSECME         Both enabled
**     FCKSM_CSECMD         Only clock switching enabled
**     FCKSM_CSDCMD         Both disabled
**
**   OSCO/RC15 function:
**     OSCIOFNC_ON          RC15
**     OSCIOFNC_OFF         OSCO or Fosc/2
**
**   RP Register Protection:
**     IOL1WAY_OFF          Unlimited Writes To RP Registers
**     IOL1WAY_ON           Write RP Registers Once
**
**   Oscillator Selection:
**     POSCMOD_EC           External clock
**     POSCMOD_XT           XT oscillator
**     POSCMOD_HS           HS oscillator
**     POSCMOD_NONE         Primary disabled
**
*/

_CONFIG2( FNOSC_PRIPLL & POSCMOD_XT )

/* The following constants can be used to set CONFIG1.
** Multiple options may be combined, as shown:
**
** _CONFIG1( OPT1_ON & OPT2_OFF & OPT3_PLL )
**
**   JTAG:
**     JTAGEN_OFF           Disabled
**     JTAGEN_ON            Enabled
**
**   Code Protect:
**     GCP_ON               Enabled
**     GCP_OFF              Disabled
**
**   Write Protect:
**     GWRP_ON              Enabled
**     GWRP_OFF             Disabled
**
**   Background Debugger:
**     BKBUG_ON             Enabled
**     BKBUG_OFF            Disabled
**
**   Clip-on Emulation mode:
**     COE_ON               Enabled
**     COE_OFF              Disabled
**
**   ICD pins select:
**     ICS_PGx3             EMUC/EMUD share PGC3/PGD3
**     ICS_PGx2             EMUC/EMUD share PGC2/PGD2
**     ICS_PGx1             EMUC/EMUD share PGC1/PGD1
**
**   Watchdog Timer:
**     FWDTEN_OFF           Disabled
**     FWDTEN_ON            Enabled
**
**   Windowed WDT:
**     WINDIS_ON            Enabled
**     WINDIS_OFF           Disabled
**
**   Watchdog prescaler:
**     FWPSA_PR32           1:32
**     FWPSA_PR128          1:128
**
**   Watchdog postscale:
**     WDTPS_PS1            1:1
**     WDTPS_PS2            1:2
**     WDTPS_PS4            1:4
**     WDTPS_PS8            1:8
**     WDTPS_PS16           1:16
**     WDTPS_PS32           1:32
**     WDTPS_PS64           1:64
**     WDTPS_PS128          1:128
**     WDTPS_PS256          1:256
**     WDTPS_PS512          1:512
**     WDTPS_PS1024         1:1,024
**     WDTPS_PS2048         1:2,048
**     WDTPS_PS4096         1:4,096
**     WDTPS_PS8192         1:8,192
**     WDTPS_PS16384        1:16,384
**     WDTPS_PS32768        1:32,768
**
*/

_CONFIG1( JTAGEN_OFF & FWDTEN_OFF )

#else
#error No device supported in "configuration_bits_config.h"
#endif

#endif /* CONFIGURATION_BITS_CONFIG_H */

#endif
