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
#include "SBUS.h"

bool sbusEnabledFlag = false;
volatile uint8_t sbusPacket[SBUS_PACKET_SIZE];
uint8_t Usart0TxBuffer[SBUS_PACKET_SIZE];
uint8_t Usart0TxBufferCount;


//------------------------------------------------------------------------------------------------------------------------
void sbusSetup()
{
//Serial.begin(100000,SERIAL_8E2);
#undef BAUD
#define BAUD 100000
#include <util/setbaud.h>
 UBRR0H = UBRRH_VALUE;
 UBRR0L = UBRRL_VALUE;
#if USE_2X
 UCSR0A |= (1 << U2X0);
#else
 UCSR0A &= ~(1 << U2X0);
#endif
// set 8E2
 UCSR0B = (0 << RXCIE0) | (0 << TXCIE0) | (0 << UDRIE0) | (0 << RXEN0) | (0 << TXEN0) | (0 << UCSZ02);
 UCSR0C = (1 << UPM01) | (1 << USBS0)| (1 << UCSZ01) | (1 << UCSZ00); // set 2 stop bits, even parity BIT
// enable TX
 UCSR0B |= (1 << TXEN0);

 uint8_t oldSREG = SREG;
 cli();

 TCCR1A = 0; // set entire TCCR1 register to 0
 TCCR1B = 0;

 OCR1A = SBUS_PACKET_RATE * 2000;  // compare match register. at pre-scalar 8 and 16 Mhz 14000 is 7 ms.  This defines the SBUS packet rate
 TCCR1B |= (1 << WGM12);  // turn on CTC mode
 TCCR1B |= (1 << CS11);  // 8 pre-scaler: 0,5 microseconds at 16mhz
 TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt

 sbusPacket[SBUS_START_BYTE] = 0x0F;
 sbusPacket[SBUS_FLAG_BYTE]  = 0x00;
 sbusPacket[SBUS_END_BYTE]   = 0x00;

 SREG = oldSREG;

 sbusEnabledFlag = true;
}

uint8_t sbusEnabled()
{
 return sbusEnabledFlag;
}

//------------------------------------------------------------------------------------------------------------------------
void sbusDisable()
{
 uint8_t oldSREG = SREG;
 cli();

// Disable uart0 TX
 UCSR0B &= ~(1 << UDRIE0);// disable Interrupt
 UCSR0B &= ~(1 << TXEN0); // disable TX
// Disable timer1
 TCCR1A = 0; // set entire TCCR1 register to 0
 TCCR1B = 0;
 TIMSK1 &= ~(1<<OCIE1A);   // Disable Interrupt Counter 1, output compare A (TIMER1_CMPA_vect)

 SREG = oldSREG;

 sbusEnabledFlag = false;
//Serial.begin(74880);
}

//------------------------------------------------------------------------------------------------------------------------
void setSbusOutputChannelValue(uint8_t channel, uint16_t value)
{

 uint8_t firstBit = 8 + (limit<uint8_t>(0,channel,15) * 11);  // Start byte plus 11 bits per channel. 16 channels
 uint8_t byteIndex = firstBit / 8;
 uint8_t bitIndex = (firstBit % 8);
 int16_t adjustedValue = limit<uint16_t>(CHANNEL_MIN_VALUE,value,CHANNEL_MAX_VALUE);
 adjustedValue = map(adjustedValue - CHANNEL_MID_VALUE,(CHANNEL_MIN_VALUE - CHANNEL_MID_VALUE),(CHANNEL_MAX_VALUE - CHANNEL_MID_VALUE),-SBUS_OFFSET_FROM_MID_POINT,SBUS_OFFSET_FROM_MID_POINT);
 adjustedValue += SBUS_MID_POINT;

 uint8_t oldSREG = SREG;
 cli();                       //Turn off interrupts so that SBUS_ISR does not run while a value is being updated
 for (uint8_t x = 0; x < 11; x++)
  {
   if (adjustedValue & 0x0001)
    {
     sbusPacket[byteIndex] |= _BV(bitIndex);
    }
   else
    {
     sbusPacket[byteIndex] &= ~_BV(bitIndex);
    }
   adjustedValue >>=  1;
   if (bitIndex == 7)
    {
     bitIndex = 0;
     byteIndex++;
    }
   else
    {
     bitIndex++;
    }
  }
 SREG = oldSREG;
}

//------------------------------------------------------------------------------------------------------------------------
void sbusSetFailsafe(bool value)
{
 uint8_t oldSREG = SREG;
 cli();                       //Turn off interrupts so that SBUS_ISR does not run while a value is being updated
 if (value)
  {
   sbusPacket[SBUS_FLAG_BYTE] |= SBUS_FAILSAFE_MASK;
  }
 else
  {
   sbusPacket[SBUS_FLAG_BYTE] &= ~SBUS_FAILSAFE_MASK;
  }
 SREG = oldSREG;
}

//------------------------------------------------------------------------------------------------------------------------
void sbusSetFrameLost(bool value)
{
 uint8_t oldSREG = SREG;
 cli();                       //Turn off interrupts so that SBUS_ISR does not run while a value is being updated
 if (value)
  {
   sbusPacket[SBUS_FLAG_BYTE] |= SBUS_FRAME_LOST_MASK;
  }
 else
  {
   sbusPacket[SBUS_FLAG_BYTE] &= ~SBUS_FRAME_LOST_MASK;
  }
 SREG = oldSREG;
}

//------------------------------------------------------------------------------------------------------------------------
void SBUS_ISR()
{
// Copy the sbusPacket to the serial output buffer and send it (8E2 at 100000 baud)

 for (uint8_t x = 0; x < SBUS_PACKET_SIZE; x++)     // code loop instead of using memcpy because sbusPacket is volatile
  {
   Usart0TxBuffer[(SBUS_PACKET_SIZE - 1) - x] = sbusPacket[x]; // last is sended first
  }
//Serial.write(&outBuf[0],sizeof(outBuf));
 Usart0TxBufferCount = SBUS_PACKET_SIZE; // set buffer size
 UCSR0B |= (1 << UDRIE0); // enable Data Register Empty Interrupt
}

// USART0 Transmit Data Register Emtpy ISR (UDR was loaded in Shift Register)
ISR(USART_UDRE_vect)
{
 if (Usart0TxBufferCount)
  {
   UDR0 = Usart0TxBuffer[--Usart0TxBufferCount];
  }
 else
  {
   UCSR0B &= ~(1 << UDRIE0); // Disable UDRE interrupt.
  }
}
