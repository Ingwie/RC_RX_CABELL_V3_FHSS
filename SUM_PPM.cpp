/*         Copyright 2020 by Ingwie (Bracame)          */
/*    This is a fork modded of RC_RX_CABELL_V3_FHSS    */
/* https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*        Compile with AVR GCC + Code::Blocks          */
/*    https://www.mediafire.com/file/cahqfrm90h7c7fy/  */
/*    Setup_OAVRCBuilder3.exe/file (Pswd : OpenAVRc)   */
/*       Old header if it is not a new file ->         */

/*
 To use this software, you must adhere to the license terms described below, and assume all responsibility for the use
 of the software.  The user is responsible for all consequences or damage that may result from using this software.
 The user is responsible for ensuring that the hardware used to run this software complies with local regulations and that
 any radio signal generated from use of this software is legal for that user to generate.  The author(s) of this software
 assume no liability whatsoever.  The author(s) of this software is not responsible for legal or civil consequences of
 using this software, including, but not limited to, any damages cause by lost control of a vehicle using this software.
 If this software is copied or modified, this disclaimer must accompany all copies.

 This PPM algorithm originated from:
             https://code.google.com/archive/p/generate-ppm-signal/
 under GNU GPL 2.  Second version dated Mar 14, 2013.
 Modifications and additions released here under GPL.
 */

/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 RC_RX_CABELL_V3_FHSS is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with RC_RX_CABELL_V3_FHSS.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "RX.h"
#include "SUM_PPM.h"

volatile int16_t ppmValueArray [CABELL_NUM_CHANNELS];
uint8_t ppmChannelCount;
bool ppmSumEnabled = false;


//------------------------------------------------------------------------------------------------------------------------
void ppmSumSetup(uint8_t channelCount)
{

 ppmChannelCount = min<uint8_t>(PPM_MaxChannels,channelCount);

 set_dir_out(PPM_OUTPUT_PIN);
 PPM_PIN_ON;  //set the PPM signal pin to the default state (off)

 uint8_t oldSREG = SREG;
 cli();
 TCCR1A = 0; // set entire TCCR1 register to 0
 TCCR1B = 0;

 OCR1A = 100;  // compare match register, change this
 TCCR1B |= (1 << WGM12);  // turn on CTC mode
 TCCR1B |= (1 << CS11);  // 8 pre-scaler: 0,5 microseconds at 16mhz
 TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
 SREG = oldSREG;
 ppmSumEnabled = true;

}

uint8_t PPMSumEnabled()
{
 return ppmSumEnabled;
}

//------------------------------------------------------------------------------------------------------------------------
void ppmSumDisable()
{

 uint8_t oldSREG = SREG;
 cli();
 TCCR1A = 0; // set entire TCCR1 register to 0
 TCCR1B = 0;
 TIMSK1 &= ~(1<<OCIE1A);   // Disable Interrupt Counter 1, output compare A (TIMER1_CMPA_vect)
 SREG = oldSREG;
 ppmSumEnabled = false;

}

//------------------------------------------------------------------------------------------------------------------------
void setPPMSumOutputChannelValue(uint8_t channel, uint16_t value)
{
 uint16_t correction = MICROSECOND_RANGE_OFFSET + (uint16_t)((float)value * MICROSECOND_RANGE_EXPANSION) ;
 value = (limit<uint16_t>(CHANNEL_MIN_VALUE,value,CHANNEL_MAX_VALUE) + correction) * TICKS_PER_US;
 if (ppmValueArray[channel] != (int16_t)value)
  {
   uint8_t oldSREG = SREG;
   cli();
   ppmValueArray[channel] = value;
   SREG = oldSREG;
  }
}

//------------------------------------------------------------------------------------------------------------------------
void SUM_PPM_ISR()
{
 static bool state = true;
 static uint8_t cur_chan_numb = 0;
 static uint16_t calc_rest = 0;

 if(state)    //start pulse
  {
   PPM_PIN_ON;
   OCR1A = PPM_PulseLen_ticks;
   state = false;
  }
 else   //end pulse and calculate when to start the next pulse
  {
   if(cur_chan_numb >= ppmChannelCount)
    {
     calc_rest = calc_rest + PPM_PulseLen_ticks;
     OCR1A = (PPM_FrLen_ticks - calc_rest);
     cur_chan_numb = 0;
     calc_rest = 0;
    }
   else
    {
     PPM_PIN_OFF;
     int16_t ppmValue = ppmValueArray[cur_chan_numb];          //Copy out of volatile storage since it is used more than once
     OCR1A = ppmValue;
     calc_rest = calc_rest + ppmValue + PPM_PulseLen_ticks;
     cur_chan_numb++;
    }
   state = true;
  }
}

