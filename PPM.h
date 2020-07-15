/*         Copyright 2020 by Ingwie (Bracame)          */
/*    This is a fork modded of RC_RX_CABELL_V3_FHSS    */
/* https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*        Compile with AVR GCC + Code::Blocks          */
/*    https://www.mediafire.com/file/cahqfrm90h7c7fy/  */
/*    Setup_OAVRCBuilder3.exe/file (Pswd : OpenAVRc)   */
/*       Old header if it is not a new file ->         */

/* PPM 8 */

#ifndef __have__PPM_h__
#define __have__PPM_h__

#include "Pins.h"
#define usToTicks(us) (us*2) // prescaler = 8
#define FRAMELENGHT   20000U // 20mS

extern uint16_t PPMValues[CABELL_NUM_PPM_CHANNELS];

void PpmSetup();
void PPMDisable();
uint8_t PPMEnabled();
void PPM_ISR();
#endif
