/*         Copyright 2020 by Ingwie (Bracame)          */
/*    This is a fork modded of RC_RX_CABELL_V3_FHSS    */
/* https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*        Compile with AVR GCC + Code::Blocks          */
/*    https://www.mediafire.com/file/cahqfrm90h7c7fy/  */
/*    Setup_OAVRCBuilder3.exe/file (Pswd : OpenAVRc)   */
/*       Old header if it is not a new file ->         */

#ifndef _UTILS_H__
#define _UTILS_H__

#include "RX.h"

#define usToFourUs(x)  (x>>2) // x/4
#define fourUsToUs(x)  (x<<2) // x*4

void init_time_machine();
uint32_t approxMillis();
uint32_t micros();

#endif
