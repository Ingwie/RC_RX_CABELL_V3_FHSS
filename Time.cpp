/*         Copyright 2020 by Ingwie (Bracame)          */
/*    This is a fork modded of RC_RX_CABELL_V3_FHSS    */
/* https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*        Compile with AVR GCC + Code::Blocks          */
/*    https://www.mediafire.com/file/cahqfrm90h7c7fy/  */
/*    Setup_OAVRCBuilder3.exe/file (Pswd : OpenAVRc)   */
/*       Old header if it is not a new file ->         */


#include "Time.h"

volatile uint32_t timer2_millis = 0;

void init_time_machine()
{
 cli();
 TCNT2 = 0;
 TCCR2A = 0;
 TCCR2B = (1<<CS22); // 64 prescaler -> 4uS tick
// enable timer 2 overflow interrupt
 TIMSK2 |= (1<<TOIE2);
 sei();
}

ISR(TIMER2_OVF_vect, ISR_NOBLOCK) // 976,5625 Hz - 1,024 mS
{
 ++timer2_millis;
}

uint32_t approxMillis()
{
 uint32_t m;
 uint8_t oldSREG = SREG;
 cli();
 m = timer2_millis;
 SREG = oldSREG;
 return m;
}

uint32_t micros()
{
 uint32_t ret = (uint32_t)(approxMillis() << 10); //X256X4
 ret |= (uint8_t)(TCNT2<<2); //X4
 return ret;
}
