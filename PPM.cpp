/*         Copyright 2020 by Ingwie (Bracame)          */
/*    This is a fork modded of RC_RX_CABELL_V3_FHSS    */
/* https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*        Compile with AVR GCC + Code::Blocks          */
/*    https://www.mediafire.com/file/cahqfrm90h7c7fy/  */
/*    Setup_OAVRCBuilder3.exe/file (Pswd : OpenAVRc)   */
/*       Old header if it is not a new file ->         */

/* PPM 8 */

#include "PPM.h"

/*
#define ROLL_PIN                 D,2  // PD2
#define PITCH_PIN                D,3  // PD3
#define THROTTLE_PIN             D,4  // PD4
#define YAW_PIN                  D,5  // PD5
#define AUX1_PIN                 D,6  // PD6
#define AUX2_PIN                 D,7  // PD7
#define AUX3_PIN                 B,0  // PB0
#define AUX4_PIN                 B,1  // PB1
*/

bool ppmEnabled = false;
uint16_t PPMValues[CABELL_NUM_PPM_CHANNELS];
uint8_t ppmChannelNum = 0;


void setPpmPinOff(uint8_t pin_map) // todo measure time elapsed to calibre
{
 uint8_t p_map = _BV(pin_map);

 PORTD &= ~(p_map << 2);
 PORTB &= ~(p_map >> 5);
}

void setPpmPinOn(uint8_t pin_map) // todo measure time elapsed to calibre
{
 uint8_t p_map = _BV(pin_map);

 PORTD |= (p_map << 2);
 PORTB |= (p_map >> 5);
}

void setPpmPinInputAll() // All input pullup
{
 DDRD &=  0b00000011;
 PORTD |= 0b11111100;
 DDRB &=  0b11111100;
 PORTB |= 0b00000011;
}

void setPpmPinOutputAll() // All output off
{
 PORTD &= 0b00000011;
 DDRD |=  0b11111100;
 PORTB &= 0b11111100;
 DDRB |=  0b00000011;
}

void PpmSetup()
{
 setPpmPinOutputAll();
 ppmEnabled = true;
 uint8_t oldSREG = SREG;
 cli();
 ppmChannelNum = 0;       // reset Num channel
 TCCR1A = 0;              // normal counting mode
 TCCR1B = _BV(CS11);      // 8 pre-scaler: 0,5 microseconds at 16mhz
 OCR1A = 0x000F;          // set short value
 TCNT1 = 0;               // clear the timer count
 TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
 TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt
 SREG = oldSREG;
}

void PPMDisable()
{
 uint8_t oldSREG = SREG;
 cli();
 TCCR1A = 0; // set entire TCCR1 register to 0
 TCCR1B = 0;
 TIMSK1 &= ~(1<<OCIE1A);   // Disable Interrupt Counter 1, output compare A (TIMER1_CMPA_vect)
 SREG = oldSREG;
 setPpmPinInputAll();
 ppmEnabled = false;
}

uint8_t PPMEnabled()
{
 return ppmEnabled;
}

void PPM_ISR()
{
 if (ppmChannelNum < CABELL_NUM_PPM_CHANNELS)
  {
   setPpmPinOn(ppmChannelNum); // channel pin ON
   if (!ppmChannelNum)
    {
     TCNT1 = 0; // reset timer
     OCR1A = 0;
    }
   else
    {
     setPpmPinOff(ppmChannelNum-1); // channel pin OFF  and increase channel number
    }
   OCR1A += usToTicks(PPMValues[ppmChannelNum++]); // load value and increase channel
  }
 else
  {
   OCR1A = usToTicks(FRAMELENGHT); // load 20mS value
   ppmChannelNum = 0; // reset
  }
}
