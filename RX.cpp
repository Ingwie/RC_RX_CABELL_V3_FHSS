/*         Copyright 2020 by Ingwie (Bracame)          */
/*    This is a fork modded of RC_RX_CABELL_V3_FHSS    */
/* https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*        Compile with AVR GCC + Code::Blocks          */
/*    https://www.mediafire.com/file/cahqfrm90h7c7fy/  */
/*    Setup_OAVRCBuilder3.exe/file (Pswd : OpenAVRc)   */
/*       Old header if it is not a new file ->         */

/*
 Copyright 2017 by Dennis Cabell
 KE8FZX

 To use this software, you must adhere to the license terms described below, and assume all responsibility for the use
 of the software.  The user is responsible for all consequences or damage that may result from using this software.
 The user is responsible for ensuring that the hardware used to run this software complies with local regulations and that
 any radio signal generated from use of this software is legal for that user to generate.  The author(s) of this software
 assume no liability whatsoever.  The author(s) of this software is not responsible for legal or civil consequences of
 using this software, including, but not limited to, any damages cause by lost control of a vehicle using this software.
 If this software is copied or modified, this disclaimer must accompany all copies.

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

#include "iface_nrf24l01.h"
#include "RX.h"
#include "Pins.h"
#include "Rx_Tx_Util.h"
#include "PPM.h"
#include "SUM_PPM.h"
#include "TestHarness.h"
#include "RSSI.h"
#include "SBUS.h"

uint16_t channelValues [CABELL_NUM_CHANNELS];
uint8_t  currentModel = 0;
uint64_t radioPipeID;
uint64_t radioNormalRxPipeID;

const uint16_t currentModelEEPROMAddress = 0; // TODO Define
const uint16_t radioPipeEEPROMAddress = currentModelEEPROMAddress + sizeof(currentModel);
const uint16_t softRebindFlagEEPROMAddress = radioPipeEEPROMAddress + sizeof(radioNormalRxPipeID);
const uint16_t failSafeChannelValuesEEPROMAddress = softRebindFlagEEPROMAddress + sizeof(uint8_t);       // uint8_t is the sifr of the rebind flag

uint16_t failSafeChannelValues[CABELL_NUM_CHANNELS];

bool throttleArmed = true;
bool bindMode = false;     // when true send bind command to cause receiver to bind enter bind mode
bool failSafeMode = false;
bool failSafeNoPulses = false;
bool packetMissed = false;
uint32_t packetInterval = DEFAULT_PACKET_INTERVAL;
uint32_t nextAutomaticChannelSwitch;
uint32_t lastRadioPacketeRecievedTime;

uint8_t radioChannel[CABELL_RADIO_CHANNELS];

volatile uint8_t currentOutputMode = 255;    // initialize to an unused mode
volatile uint8_t nextOutputMode = 255;       // initialize to an unused mode
volatile uint8_t channelsRecieved = 0;
volatile bool packetReady = false;

bool telemetryEnabled = false;
int16_t analogValue[2] = {0,0};

uint16_t initialTelemetrySkipPackets = 0;

uint8_t currentChannel = CABELL_RADIO_MIN_CHANNEL_NUM;  // Initializes the channel sequence.

#ifdef TEST_HARNESS
 TestHarness testOut;
#endif

RSSI rssi;

//--------------------------------------------------------------------------------------------------------------------------
void setupReciever()
{
 uint8_t softRebindFlag;

 softRebindFlag = eeprom_read_byte((const uint8_t *)softRebindFlagEEPROMAddress);
 eeprom_read_block(&radioNormalRxPipeID, (const uint8_t *)radioPipeEEPROMAddress, 8);
 currentModel = eeprom_read_byte((const uint8_t *)currentModelEEPROMAddress);

 if (softRebindFlag == BOUND_WITH_FAILSAFE_NO_PULSES)
  {
   softRebindFlag = DO_NOT_SOFT_REBIND;
   failSafeNoPulses = true;
  }

 if ((get_input(BIND_BUTTON_PIN) == 0) || (softRebindFlag != DO_NOT_SOFT_REBIND))
  {
   bindMode = true;
   radioPipeID = CABELL_BIND_RADIO_ADDR;
   //Serial.println (F("Bind Mode "));
   pin_high(LED_PIN);      // Turn on LED to indicate bind mode
   radioNormalRxPipeID = (uint64_t)0x01<<43;   // This is a number bigger than the max possible pipe ID, which only uses 40 bits.  This makes sure the bind routine writes to EEPROM
  }
 else
  {
   bindMode = false;
   radioPipeID = radioNormalRxPipeID;
  }

 getChannelSequence (radioChannel, CABELL_RADIO_CHANNELS, radioPipeID);


#ifdef TEST_HARNESS
//Serial.println(F("Compiled for transceiver testing. Output Disabled.  Use LCD."));
#endif

 initializeRadio();
 setTelemetryPowerMode(CABELL_OPTION_MASK_MAX_POWER_OVERRIDE);

 packetReady = false;

 outputFailSafeValues(false);   // initialize default values for output channels

//Serial.print(F("Radio ID: "));//Serial.print((uint32_t)(radioPipeID>>32)); //Serial.print(F("    "));//Serial.println((uint32_t)((radioPipeID<<32)>>32));

//Serial.print(F("Current Model Number: "));//Serial.println(currentModel);
#ifdef TEST_HARNESS
 testOut.init();
//Serial.println(F("Operating as a test harness.  Output disabled."));
#endif

 setNextRadioChannel(true);

//setup NRF IRQ pin change interrupt
 uint8_t oldSREG = SREG;
 cli();    // switch interrupts off while messing with their settings
 PCICR = 0x02;          // Enable PCINT1 interrupt
 PCMSK1 = mask(RADIO_IRQ_PIN);
 SREG = oldSREG;

 nextAutomaticChannelSwitch = micros() + RESYNC_WAIT_MICROS;
 lastRadioPacketeRecievedTime = approxMillis() - (uint32_t)RESYNC_TIME_OUT;

}

//--------------------------------------------------------------------------------------------------------------------------
ISR(PCINT1_vect, ISR_NOBLOCK)
{
 if (!(get_input(RADIO_IRQ_PIN)))     // pulled low when packet is received
  {
   packetReady = true;
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void outputChannels()
{
#ifndef TEST_HARNESS
 if (!bindMode)
  {
   if (failSafeNoPulses && failSafeMode)
    {
     nextOutputMode = 255;   // set to unused output mode
    }

   if (!throttleArmed)
    {
     channelValues[THROTTLE_CHANNEL] = THROTTLE_DISARM_VALUE;     // Safety precaution.  Min throttle if not armed
    }

   bool firstPacketOnMode = false;

   if (currentOutputMode != nextOutputMode)     // If new mode, turn off all modes
    {
     firstPacketOnMode = true;
     if (PPMEnabled()) PPMDisable();
     if (PPMSumEnabled()) ppmSumDisable();
     if (sbusEnabled()) sbusDisable();
    }

   if (nextOutputMode == CABELL_RECIEVER_OUTPUT_PWM)
    {
     outputPPM();                               // Do this first so we have something to send when PWM enabled
     if (firstPacketOnMode)                     // First time through attach pins to start output
      {
       if (!PPMEnabled())
        {
         PpmSetup();
        }
      }
    }

   if (nextOutputMode == CABELL_RECIEVER_OUTPUT_PPM)
    {
     outputSumPPM();                           // Do this first so we have something to send when PPM enabled
     if (firstPacketOnMode)
      {
       if (!PPMSumEnabled())
        {
         ppmSumSetup(RX_NUM_CHANNELS);
        }
      }
    }

   if (nextOutputMode == CABELL_RECIEVER_OUTPUT_SBUS)
    {
     outputSbus();                           // Do this first so we have something to send when SBUS enabled
     if (firstPacketOnMode)
      {
       if (!sbusEnabled())
        {
         sbusSetup();
        }
      }
    }
   currentOutputMode = nextOutputMode;
  }
#endif
}

//--------------------------------------------------------------------------------------------------------------------------
void setNextRadioChannel(bool missedPacket)
{
 /* Clear IRQ */
 NRF24L01_WriteReg(NRF24L01_07_STATUS, _BV(NRF24L01_07_RX_DR)|_BV(NRF24L01_07_TX_DS)|_BV(NRF24L01_07_MAX_RT));
 /* Set TX, CRC 2 bytes, no IRQ enabled */
 NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_PWR_UP)|_BV(NRF24L01_00_CRCO)|_BV(NRF24L01_00_EN_CRC)
                   |_BV(NRF24L01_00_MASK_MAX_RT)|_BV(NRF24L01_00_MASK_TX_DS)|_BV(NRF24L01_00_MASK_RX_DR));
 _delay_us(130);

 NRF24L01_FlushRx();

 uint32_t expectedTransmitCompleteTime = 0;
 if (telemetryEnabled)
  {
   if (initialTelemetrySkipPackets >= INITIAL_TELEMETRY_PACKETS_TO_SKIP)    // don't send the first 500 telemetry packets to avoid annoying warnings at startup
    {
     expectedTransmitCompleteTime = sendTelemetryPacket();
    }
   else
    {
     initialTelemetrySkipPackets++;
    }
  }

// todo use missedPacket ?;

 currentChannel = getNextChannel (radioChannel, CABELL_RADIO_CHANNELS, currentChannel);

 if (expectedTransmitCompleteTime != 0)
  {
   // Wait here for the telemetry packet to finish transmitting
   int32_t waitTimeLeft = (int32_t)(expectedTransmitCompleteTime - micros());
   if (waitTimeLeft > 0)
    {
     _delay_us(waitTimeLeft);
    }
  }
 pin_high(RADIO_CE_PIN); // LNA in on when CE is Vcc

 NRF24L01_WriteReg(NRF24L01_05_RF_CH, currentChannel); // Set channel

 packetReady = false;

 /* Clear IRQ */
 NRF24L01_WriteReg(NRF24L01_07_STATUS, _BV(NRF24L01_07_RX_DR)|_BV(NRF24L01_07_TX_DS)|_BV(NRF24L01_07_MAX_RT));
 /* Set RX, CRC 2 bytes, only RX IRQ enabled */
 NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_PWR_UP)|_BV(NRF24L01_00_PRIM_RX)|_BV(NRF24L01_00_CRCO)
                   |_BV(NRF24L01_00_EN_CRC)|_BV(NRF24L01_00_MASK_MAX_RT)|_BV(NRF24L01_00_MASK_TX_DS));
 _delay_us(130);
 NRF24L01_FlushTx();   //just in case things got hung up*/
}

//--------------------------------------------------------------------------------------------------------------------------
bool getPacket()
{
 static uint32_t lastPacketTime = 0;
 static bool inititalGoodPacketRecieved = false;
 static bool hoppingLockedIn = false;
 static uint16_t sequentialHitCount = 0;
 static uint16_t sequentialMissCount = 0;
 static bool powerOnLock = false;
 bool goodPacket_rx = false;
 bool strongSignal = false;

// Wait for the radio to get a packet, or the timeout for the current radio channel occurs
 if (!packetReady)
  {
   if ((int32_t)(micros() - nextAutomaticChannelSwitch) >= 0 )        // if timed out the packet was missed, go to the next channel
    {
     packetMissed = true;
     sequentialHitCount = 0;
     sequentialMissCount++;
     rssi.miss();
#ifdef TEST_HARNESS
     testOut.miss();
#endif
     setNextRadioChannel(true);       //true indicates that packet was missed
     if ((int32_t)(nextAutomaticChannelSwitch - lastRadioPacketeRecievedTime) > ((int32_t)RESYNC_TIME_OUT))    // if a uint32_t time passed, increase timeout duration to re-sync with the TX
      {
       telemetryEnabled = false;
#ifdef TEST_HARNESS
       testOut.reSync();
#else
       if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
        {
         //Serial.println(F("Re-sync Attempt"));
        }
#endif
       hoppingLockedIn = false;
       sequentialHitCount = 0;
       sequentialMissCount = 0;
       packetInterval = DEFAULT_PACKET_INTERVAL;
       initialTelemetrySkipPackets = 0;
       nextAutomaticChannelSwitch += RESYNC_WAIT_MICROS;
      }
     else
      {
       nextAutomaticChannelSwitch += packetInterval;
      }
     checkFailsafeDisarmTimeout(lastPacketTime,inititalGoodPacketRecieved);   // at each timeout, check for failsafe and disarm.  When disarmed TX must send min throttle to re-arm.
    }
  }
 else
  {
   lastRadioPacketeRecievedTime = micros();   //Use this time to calculate the next expected packet so when we miss packets we can change channels
   if (!powerOnLock)
    {
     // save this now while the value is latched.  To save loop time only do this before initial lock as the initial lock process is the only thing that needs this
     strongSignal = (NRF24L01_ReadReg(NRF24L01_09_CD) & 0x01);
    }
   goodPacket_rx = readAndProcessPacket();
   nextAutomaticChannelSwitch = lastRadioPacketeRecievedTime + packetInterval + INITIAL_PACKET_TIMEOUT_ADD;  // must ne set after readAndProcessPacket because packetInterval may get adjusted
   if (!powerOnLock && !strongSignal)
    {
     // During the initial power on lock process only consider the packet good if the signal was strong (better than -64 DBm)
     goodPacket_rx = false;
    }
   if (goodPacket_rx)
    {
     sequentialHitCount++;
     sequentialMissCount = 0;
     inititalGoodPacketRecieved = true;
     lastPacketTime = micros();
     failSafeMode = false;
     packetMissed = false;
     rssi.hit();
#ifdef TEST_HARNESS
     testOut.hit();
#endif
    }
   else
    {
     sequentialHitCount = 0;
     sequentialMissCount++;
     rssi.badPacket();
     packetMissed = true;
#ifdef TEST_HARNESS
     testOut.badPacket();
#endif
    }
  }

// Below tries to detect when a false lock occurs and force a re-sync when detected in order to get a good lock.
// This happens when while syncing the NRF24L01 successfully receives a packet on an adjacent channel to the current channel,
// which locks the algorithm into the wrong place in the channel progression.  If the module continues to occasionally receive a
// packet like this, a re-sync does not happen, but the packet success rate is very poor.  This manifests as studdering control surfaces.
// This seems to only happen when the TX is close to the RX as the strong signal swamps the RX module.
// Checking for 5 good packets in a row to confirm lock, or 5 misses to force re-sync.
//
// For the initial lock when the receiver is powered on, the rule is much more stringent to get a lock, and all packets are flagged bad until
// the power on lock is obtained.  This is so that the model cannot be controlled until the initial lock is obtained.
// This is only for the first lock.  A re-sync is less stringent so that if lock is lost for a model in flight then control is easier to re-establish.
// Also, a re-sync that is not yet locked are considered good packets so that a weak re-sync can still control the model.

 if (!hoppingLockedIn)
  {
   if (!powerOnLock)
    {
     goodPacket_rx = false;           // Always consider a bad packet until initial lock is obtained so no control signals are output.
     if (sequentialHitCount > (CABELL_RADIO_CHANNELS * 5) )       // Ensure strong signal on all channels
      {
       powerOnLock = true;
       hoppingLockedIn = true;
       if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
        {
         //Serial.println(F("Initial Signal Locked"));
        }
      }
    }
   else if (sequentialHitCount > 5)
    {
     hoppingLockedIn = true;
     if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
      {
       //Serial.println(F("Signal Locked"));
      }
    }
   if ((sequentialMissCount > 5) || (sequentialMissCount + sequentialHitCount > 100))    // if more tnan 5 misses in a row assume it is a bad lock, or if after 100 packets there is still no lock
    {
     //if this happens then there is a bad lock and we should try to sync again.
     lastRadioPacketeRecievedTime = approxMillis() - (uint32_t)RESYNC_TIME_OUT;
     nextAutomaticChannelSwitch = approxMillis() + RESYNC_WAIT_MICROS;
     telemetryEnabled = false;
     setNextRadioChannel(true);   //Getting the next channel ensures radios are flushed and properly waiting for a packet
     if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
      {
       //Serial.println(F("False Signal Lock"));
      }
    }
  }
 return goodPacket_rx;
}

//--------------------------------------------------------------------------------------------------------------------------
void checkFailsafeDisarmTimeout(uint32_t lastPacketTime,bool inititalGoodPacketRecieved)
{
 uint32_t holdMicros = micros();

 if ((int32_t)(holdMicros - lastPacketTime)  > ((int32_t)RX_CONNECTION_TIMEOUT))
  {
   outputFailSafeValues(true);
  }

 if (((int32_t)(holdMicros - lastPacketTime) >  ((int32_t)RX_DISARM_TIMEOUT)) || (!inititalGoodPacketRecieved && ((int32_t)(holdMicros - lastPacketTime)  > ((int32_t)RX_DISARM_TIMEOUT)) ) )
  {
   if (throttleArmed)
    {
     if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
      {
       //Serial.println(F("Disarming throttle"));
      }
     throttleArmed = false;
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void outputPPM()
{
 for(uint8_t x = 0; x < CABELL_NUM_PPM_CHANNELS; x++)
  {
   channelValues[x] = limit<uint16_t>(channelValues[x],CHANNEL_MIN_VALUE,CHANNEL_MAX_VALUE);
   uint8_t oldSREG = SREG;
   cli();
   PPMValues[x] = channelValues[x];
   SREG = oldSREG;
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void outputSumPPM()    // output as AETR
{
 int adjusted_x;
 for(uint8_t x = 0; x < CABELL_NUM_CHANNELS ; x++)    //
  {
   //set adjusted_x to be in AETR order
   switch (x)
    {
    case 0:
     adjusted_x = ROLL_CHANNEL;
     break;
    case 1:
     adjusted_x = PITCH_CHANNEL;
     break;
    case 2:
     adjusted_x = THROTTLE_CHANNEL;
     break;
    case 3:
     adjusted_x = YAW_CHANNEL;
     break;
    default:
     adjusted_x = x;
    }

   setPPMSumOutputChannelValue(x, channelValues[adjusted_x]);
   ////Serial.print(channelValues[x]); //Serial.print("\t");
  }
////Serial.println();
}

//--------------------------------------------------------------------------------------------------------------------------
void outputSbus()    // output as AETR
{
 int adjusted_x;
 for(uint8_t x = 0; x < CABELL_NUM_CHANNELS ; x++)    //
  {

   //set adjusted_x to be in AETR order
   switch (x)
    {
    case 0:
     adjusted_x = ROLL_CHANNEL;
     break;
    case 1:
     adjusted_x = PITCH_CHANNEL;
     break;
    case 2:
     adjusted_x = THROTTLE_CHANNEL;
     break;
    case 3:
     adjusted_x = YAW_CHANNEL;
     break;
    default:
     adjusted_x = x;
    }

   setSbusOutputChannelValue(x, channelValues[adjusted_x]);
  }
 sbusSetFailsafe(failSafeMode);
 sbusSetFrameLost(packetMissed);
}

//--------------------------------------------------------------------------------------------------------------------------
void outputFailSafeValues(bool callOutputChannels)
{

 loadFailSafeDefaultValues();

 for (uint8_t x = 0; x < CABELL_NUM_CHANNELS; x++) // TODO memcpy
  {
   channelValues[x] = failSafeChannelValues[x];
  }

 if (!failSafeMode)
  {
#ifdef TEST_HARNESS
   testOut.failSafe();
#else
   if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
    {
     //Serial.println(F("Failsafe"));
    }
#endif
   failSafeMode = true;
  }
 if (callOutputChannels)
  {
   outputChannels();
  }
}

//--------------------------------------------------------------------------------------------------------------------------
#ifndef TEST_HARNESS
ISR(TIMER1_COMPA_vect)
{
 if (currentOutputMode == CABELL_RECIEVER_OUTPUT_PWM)
  PPM_ISR();
 else if (currentOutputMode == CABELL_RECIEVER_OUTPUT_PPM)
  SUM_PPM_ISR();
 else if (currentOutputMode == CABELL_RECIEVER_OUTPUT_SBUS)
  SBUS_ISR();
}
#endif

//--------------------------------------------------------------------------------------------------------------------------
void unbindReciever()
{
// Reset all of flash memory to unbind receiver
 uint8_t value = 0xFF;
 if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
  {
   //Serial.print(F("Overwriting flash with value "));//Serial.println(value, HEX);
  }
 for (int x = 0; x < 1024; x++)
  {
   eeprom_write_byte ((uint8_t *)x,value);
  }
 if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
  {
   //Serial.println(F("Reciever un-bound.  Reboot to enter bind mode"));
  }
 outputFailSafeValues(true);
 while (true)                                             // Flash LED forever indicating unbound
  {
   pin_toggle(LED_PIN);
   _delay_ms(250);                                            // Fast LED flash
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void bindReciever(uint8_t modelNum, uint16_t tempHoldValues[], CABELL_RxTxPacket_t::RxMode_t RxMode)
{
// new radio address is in channels 11 to 15
 uint64_t newRadioPipeID = (((uint64_t)(tempHoldValues[11]-1000)) << 32) +
                           (((uint64_t)(tempHoldValues[12]-1000)) << 24) +
                           (((uint64_t)(tempHoldValues[13]-1000)) << 16) +
                           (((uint64_t)(tempHoldValues[14]-1000)) << 8)  +
                           (((uint64_t)(tempHoldValues[15]-1000)));    // Address to use after binding

 if ((modelNum != currentModel) || (radioNormalRxPipeID != newRadioPipeID))
  {
   eeprom_write_byte((uint8_t *)currentModelEEPROMAddress,modelNum);
   radioNormalRxPipeID = newRadioPipeID;
   eeprom_write_block (&radioNormalRxPipeID,(uint8_t *)radioPipeEEPROMAddress,8);
   //Serial.print(F("Bound to new TX address for model number "));//Serial.println(modelNum);
   if (RxMode == CABELL_RxTxPacket_t::RxMode_t::bindFalesafeNoPulse)
    {
     eeprom_write_byte((uint8_t *)softRebindFlagEEPROMAddress,(uint8_t)BOUND_WITH_FAILSAFE_NO_PULSES);
    }
   else
    {
     eeprom_write_byte((uint8_t *)softRebindFlagEEPROMAddress,(uint8_t)DO_NOT_SOFT_REBIND);
    }
   pin_low(LED_PIN); // Turn off LED to indicate successful bind
   setFailSafeDefaultValues();
   outputFailSafeValues(true);
   //Serial.println(F("Reciever bound.  Reboot to enter normal mode"));
   while (true)                                             // Flash LED forever indicating bound
    {
     pin_toggle(LED_PIN);
     _delay_ms(2000);                                            // Slow flash
    }
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void setFailSafeDefaultValues()
{
 uint16_t defaultFailSafeValues[CABELL_NUM_CHANNELS];
 for (int x = 0; x < CABELL_NUM_CHANNELS; x++)
  {
   defaultFailSafeValues[x] = CHANNEL_MID_VALUE;
  }
 defaultFailSafeValues[THROTTLE_CHANNEL] = THROTTLE_DISARM_VALUE;           // Throttle should always be the min value when failsafe}
 setFailSafeValues(defaultFailSafeValues);
}

//--------------------------------------------------------------------------------------------------------------------------
void loadFailSafeDefaultValues()
{
 eeprom_read_block((void*)failSafeChannelValues, (const void*)failSafeChannelValuesEEPROMAddress, CABELL_NUM_CHANNELS*2);
 for (uint8_t x = 0; x < CABELL_NUM_CHANNELS; x++)
  {
   if (failSafeChannelValues[x] < CHANNEL_MIN_VALUE || failSafeChannelValues[x] > CHANNEL_MAX_VALUE)      // Make sure failsafe values are valid
    {
     failSafeChannelValues[x] = CHANNEL_MID_VALUE;
    }
  }
 failSafeChannelValues[THROTTLE_CHANNEL] = THROTTLE_DISARM_VALUE;     // Throttle should always be the min value when failsafe
}

//--------------------------------------------------------------------------------------------------------------------------
void setFailSafeValues(uint16_t newFailsafeValues[])
{
 for (int x = 0; x < CABELL_NUM_CHANNELS; x++)
  {
   failSafeChannelValues[x] = newFailsafeValues[x];
  }
 failSafeChannelValues[THROTTLE_CHANNEL] = THROTTLE_DISARM_VALUE;           // Throttle should always be the min value when failsafe}
 eeprom_write_block((const void*)failSafeChannelValues, (void*)failSafeChannelValuesEEPROMAddress, CABELL_NUM_CHANNELS*2);
 if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
  {
   //Serial.println(F("Fail Safe Values Set"));
  }
}

//--------------------------------------------------------------------------------------------------------------------------
bool validateChecksum(CABELL_RxTxPacket_t const& packet, uint8_t maxPayloadValueIndex)
{
//caculate checksum and validate
 uint16_t packetSum = packet.modelNum + packet.option + packet.RxMode + packet.reserved;

 for (int x = 0; x < maxPayloadValueIndex; x++)
  {
   packetSum = packetSum +  packet.payloadValue[x];
  }

 if (packetSum != ((((uint16_t)packet.checkSum_MSB) <<8) + (uint16_t)packet.checkSum_LSB))
  {
   return false;       // don't take packet if checksum bad
  }
 else
  {
   return true;
  }
}

//--------------------------------------------------------------------------------------------------------------------------
bool readAndProcessPacket()      //only call when a packet is available on the radio
{
 CABELL_RxTxPacket_t RxPacket;

// Fetch the payload
 NRF24L01_ReadPayload((uint8_t*)&RxPacket, sizeof(RxPacket));
 /* Clear IRQ */
 NRF24L01_WriteReg(NRF24L01_07_STATUS, _BV(NRF24L01_07_RX_DR)|_BV(NRF24L01_07_TX_DS)|_BV(NRF24L01_07_MAX_RT));

 int tx_channel = RxPacket.reserved & CABELL_RESERVED_MASK_CHANNEL;
 if (tx_channel != 0 )
  {
   currentChannel = tx_channel;
  }
 setNextRadioChannel(false);                   // Also sends telemetry if in telemetry mode.  Doing this as soon as possible to keep timing as tight as possible
// False indicates that packet was not missed

// Remove 8th bit from RxMode because this is a toggle bit that is not included in the checksum
// This toggle with each xmit so consecutive payloads are not identical.  This is a work around for a reported bug in clone NRF24L01 chips that mis-took this case for a re-transmit of the same packet.
 uint8_t* p = (uint8_t*)(&RxPacket.RxMode);
 *p &= 0x7F;  //ensure 8th bit is not set.  This bit is not included in checksum

// putting this after setNextRadioChannel will lag by one telemetry packet, but by doing this the telemetry can be sent sooner, improving the timing
 telemetryEnabled = (RxPacket.RxMode==CABELL_RxTxPacket_t::RxMode_t::normalWithTelemetry)?true:false;

 bool packet_rx = false;
 uint16_t tempHoldValues [CABELL_NUM_CHANNELS];
 uint8_t channelReduction = limit((RxPacket.option & CABELL_OPTION_MASK_CHANNEL_REDUCTION),0,CABELL_NUM_CHANNELS-CABELL_MIN_CHANNELS);  // Must be at least 4 channels, so cap at 12
 uint8_t packetSize = sizeof(RxPacket) - ((((channelReduction - (channelReduction%2))/ 2)) * 3);      // reduce 3 bytes per 2 channels, but not last channel if it is odd
 uint8_t maxPayloadValueIndex = sizeof(RxPacket.payloadValue) - (sizeof(RxPacket) - packetSize);
 channelsRecieved = CABELL_NUM_CHANNELS - channelReduction;

 if (telemetryEnabled)    // putting this after setNextRadioChannel will lag by one telemetry packet, but by doing this the telemetry can be sent sooner, improving the timing
  {
   setTelemetryPowerMode(RxPacket.option);
   packetInterval = DEFAULT_PACKET_INTERVAL + (limit<int16_t>((channelsRecieved - 6),0,10 ) * (int16_t)100);   // increase packet period by 100 us for each channel over 6
  }
 else
  {
   packetInterval = DEFAULT_PACKET_INTERVAL;
  }

 packet_rx = validateChecksum(RxPacket, maxPayloadValueIndex);

 if (packet_rx)
  packet_rx = decodeChannelValues(RxPacket, channelsRecieved, tempHoldValues);

 if (packet_rx)
  packet_rx = processRxMode (RxPacket.RxMode, RxPacket.modelNum, tempHoldValues);    // If bind or unbind happens, this will never return.

// if packet is good, copy the channel values
 if (packet_rx)
  {
   nextOutputMode = (RxPacket.option & CABELL_OPTION_MASK_RECIEVER_OUTPUT_MODE) >> CABELL_OPTION_SHIFT_RECIEVER_OUTPUT_MODE;
   for ( int b = 0 ; b < CABELL_NUM_CHANNELS ; b ++ )
    {
     channelValues[b] =  (b < channelsRecieved) ? tempHoldValues[b] : CHANNEL_MID_VALUE;   // use the mid value for channels not received.
    }
  }
 else
  {
   if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
    {
     //Serial.println("RX Pckt Err");    // Don't use F macro here.  Want this to be fast as it is in the main loop logic
    }
  }

 return packet_rx;
}

//--------------------------------------------------------------------------------------------------------------------------
bool processRxMode (uint8_t RxMode, uint8_t modelNum, uint16_t tempHoldValues[])
{
 static bool failSafeValuesHaveBeenSet = false;
 bool packet_rx = true;

// fail safe settings can come in on a failsafe packet, but also use a normal packed if bind mode button is pressed after start up
 if (failSafeButtonHeld())
  {
   if (RxMode == CABELL_RxTxPacket_t::RxMode_t::normal || RxMode == CABELL_RxTxPacket_t::RxMode_t::normalWithTelemetry)
    {
     RxMode = CABELL_RxTxPacket_t::RxMode_t::setFailSafe;
    }
  }

 switch (RxMode)
  {
  case CABELL_RxTxPacket_t::RxMode_t::bindFalesafeNoPulse   :
  case CABELL_RxTxPacket_t::RxMode_t::bind                  :
   if (bindMode)
    {
     bindReciever(modelNum, tempHoldValues, (CABELL_RxTxPacket_t::RxMode_t)RxMode);
    }
   else
    {
     if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
      {
       //Serial.println(F("Bind command detected but receiver not in bind mode"));
      }
     packet_rx = false;
    }
   break;

  case CABELL_RxTxPacket_t::RxMode_t::setFailSafe :
   if (modelNum == currentModel)
    {
     pin_high(LED_PIN);
     if (!failSafeValuesHaveBeenSet)        // only set the values first time through
      {
       failSafeValuesHaveBeenSet = true;
       setFailSafeValues(tempHoldValues);
      }
    }
   else
    {
     packet_rx = false;
     if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
      {
       //Serial.println(F("Wrong Model Number"));
      }
    }
   break;

  case CABELL_RxTxPacket_t::RxMode_t::normalWithTelemetry :
  case CABELL_RxTxPacket_t::RxMode_t::normal :
   if (modelNum == currentModel)
    {
     pin_low(LED_PIN);
     failSafeValuesHaveBeenSet = false;             // Reset when not in setFailSafe mode so next time failsafe is to be set it will take
     if (!throttleArmed && (tempHoldValues[THROTTLE_CHANNEL] <= THROTTLE_DISARM_VALUE + 10) && (tempHoldValues[THROTTLE_CHANNEL] >= THROTTLE_DISARM_VALUE - 10))
      {
       if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
        {
         //Serial.println("Throttle Armed");             // Don't use F macro here.  Want this to be fast as it is in the main loop logic
        }
       throttleArmed = true;
      }
    }
   else
    {
     packet_rx = false;
     if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
      {
       //Serial.println(F("Wrong Model Number"));
      }
    }
   break;

  case CABELL_RxTxPacket_t::RxMode_t::unBind :
   if (modelNum == currentModel)
    {
     unbindReciever();
    }
   else
    {
     packet_rx = false;
     if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
      {
       //Serial.println(F("Wrong Model Number"));
      }
    }
   break;
  default :
   if (currentOutputMode != CABELL_RECIEVER_OUTPUT_SBUS)
    {
     //Serial.println(F("Unknown RxMode"));
    }
   packet_rx = false;
   break;
  }
 return packet_rx;
}

//--------------------------------------------------------------------------------------------------------------------------
bool decodeChannelValues(CABELL_RxTxPacket_t const& RxPacket, uint8_t channelsRecieved, uint16_t tempHoldValues[])
{
// decode the 12 bit numbers to temp array.
 bool packet_rx = true;
 int payloadIndex = 0;

 for ( int b = 0 ; (b < channelsRecieved); b ++ )
  {
   tempHoldValues[b] = RxPacket.payloadValue[payloadIndex];
   payloadIndex++;
   tempHoldValues[b] |= ((uint16_t)RxPacket.payloadValue[payloadIndex]) <<8;
   if (b % 2)       //channel number is ODD
    {
     tempHoldValues[b] = tempHoldValues[b]>>4;
     payloadIndex++;
    }
   else             //channel number is EVEN
    {
     tempHoldValues[b] &= 0x0FFF;
    }
   if ((tempHoldValues[b] > CHANNEL_MAX_VALUE) || (tempHoldValues[b] < CHANNEL_MIN_VALUE))
    {
     packet_rx = false;   // throw out entire packet if any value out of range
    }
  }
 return packet_rx;
}

//--------------------------------------------------------------------------------------------------------------------------
uint32_t sendTelemetryPacket()
{
 pin_low(RADIO_CE_PIN); // PA in on when CE is 0V

 static int8_t packetCounter = 0;  // this is only used for toggling bit
 uint8_t sendPacket[4] = {CABELL_RxTxPacket_t::RxMode_t::telemetryResponse};

 packetCounter++;
 sendPacket[0] &= 0x7F;                 // clear 8th bit
 sendPacket[0] |= packetCounter<<7;     // This causes the 8th bit of the first byte to toggle with each xmit so consecutive payloads are not identical.  This is a work around for a reported bug in clone NRF24L01 chips that mis-took this case for a re-transmit of the same packet.
 sendPacket[1] = rssi.getRSSI();
 sendPacket[2] = analogValue[0]/4;      // Send a 8 bit value (0 to 255) of the analog input.  Can be used for LiPo voltage or other analog input for telemetry
 sendPacket[3] = analogValue[1]/4;      // Send a 8 bit value (0 to 255) of the analog input.  Can be used for LiPo voltage or other analog input for telemetry

 uint8_t packetSize =  sizeof(sendPacket);

//TMRh20 -> Per the documentation, we want to set PTX Mode when not listening. Then all we do is write data and set CE high
//In this mode, if we can keep the FIFO buffers loaded, packets will transmit immediately (no 130us _delay_ms)
//Otherwise we enter Standby-II mode, which is still faster than standby mode
//Also, we remove the need to keep writing the config register over and over and delaying for 150 us each time if sending a stream of data
//Ingwie -> great !
 NRF24L01_WritePayload(sendPacket, packetSize);
// send code :
 pin_high(RADIO_CE_PIN); // Send
 _delay_us(15); // 10uS minimum
 pin_low(RADIO_CE_PIN); // PA in on when CE is 0V

// calculate transmit time based on packet size and data rate of 1MB per sec
// This is done because polling the status register during xmit to see when xmit is done causes issues sometimes.
// bits = packet-size * 8  +  73 bits overhead
// at 250 kbps per sec, one bit is 4 uS
// then add 140 uS which is 130 uS to begin the xmit and 10 uS fudge factor
// Add this to micros() to return when the transmit is expected to be complete
 return micros() + (((((uint32_t)packetSize * 8ul)  +  73ul) * 4ul) + 140ul) ;
}


//--------------------------------------------------------------------------------------------------------------------------
// based on ADC Interrupt example from https://www.gammon.com.au/adc
void ADC_Processing()               //Reads ADC value then configures next conversion. Alternates between pins A6 and A7
{
 static uint8_t adcPin = TELEMETRY_ANALOG_INPUT_1;

 if (bit_is_clear(ADCSRA, ADSC))
  {
   analogValue[(adcPin==TELEMETRY_ANALOG_INPUT_1) ? 0 : 1] = ADC;

   adcPin = (adcPin==TELEMETRY_ANALOG_INPUT_2) ? TELEMETRY_ANALOG_INPUT_1 : TELEMETRY_ANALOG_INPUT_2;   // Choose next pin to read

   ADCSRA =  _BV (ADEN);                      // turn ADC on
   ADCSRA |= _BV (ADPS0) |  _BV (ADPS1) | _BV (ADPS2);  // Pre-scaler of 128
   ADMUX  =  _BV (REFS0) | (adcPin & 0x07);    // AVcc and select input port

   ADCSRA |= _BV (ADSC);   //Start next conversion
  }
}

//--------------------------------------------------------------------------------------------------------------------------
bool failSafeButtonHeld()
{
// use the bind button because bind mode is only checked at startup.  Once RX is started and not in bind mode it is the set failsafe button

 static uint32_t heldTriggerTime = 0;

 if(!bindMode && !get_input(BIND_BUTTON_PIN))    // invert because pin is pulled up so low means pressed
  {
   if (heldTriggerTime == 0)
    {
     heldTriggerTime = micros() + 1000000ul;   // Held state achieved after button is pressed for 1 second
    }
   if ((int32_t)(micros() - heldTriggerTime) >= 0)
    {
     return true;
    }
   else
    {
     return false;
    }
  }
 heldTriggerTime = 0;
 return false;
}

//--------------------------------------------------------------------------------------------------------------------------
void setTelemetryPowerMode(uint8_t option)
{
// Set transmit power to max or high based on the option byte in the incoming packet.
// This should set the power the same as the transmitter module

 static uint8_t prevPower = NRF24L01_PA_MIN;
 uint8_t newPower;
 if ((option & CABELL_OPTION_MASK_MAX_POWER_OVERRIDE) == 0)
  {
   newPower = NRF24L01_PA_HIGH;
  }
 else
  {
   newPower = NRF24L01_PA_MAX;
  }
 if (newPower != prevPower)
  {
   NRF24L01_SetPower(newPower);
   prevPower = newPower;
  }
}

//--------------------------------------------------------------------------------------------------------------------------
void initializeRadio()
{
 pin_high(RADIO_CE_PIN); // Chip not enabled (PA is ON)
 _delay_ms(200); // 100 mS minimum
 NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_PWR_UP)); // Power up
 _delay_ms(2); // 1.5 mS minimum
 NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70); // Reset status
 /* Set RX, CRC 2 bytes, only RX IRQ enabled */
 NRF24L01_WriteReg(NRF24L01_00_CONFIG, _BV(NRF24L01_00_PWR_UP)|_BV(NRF24L01_00_PRIM_RX)|_BV(NRF24L01_00_CRCO)
                   |_BV(NRF24L01_00_EN_CRC)|_BV(NRF24L01_00_MASK_MAX_RT)|_BV(NRF24L01_00_MASK_TX_DS));
 NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00); // No Auto Acknowledgment on all data pipes
 NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01); // Enable only data Pipe 0
 NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x03); // 5 bytes adress
 NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x5F);	// but no retransmits
 NRF24L01_WriteReg(NRF24L01_05_RF_CH, 0x00); // start out on a channel we don't use so we don't start receiving packets yet.  It will get changed when the looping starts
 NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x01); // reset value and Set obsolete value for hight LNA gain..
 NRF24L01_SetBitrate(NRF24L01_BR_250K); // ... set slower data rate gives better range/reliability and ...
 NRF24L01_SetPower(NRF24L01_PA_MIN); // ... set -18dBm output power (without PA) at start
 /* Set RX and TX address */
 uint64_t radioPipeID_tele = ~radioPipeID;
 NRF24L01_WriteRegisterMulti(NRF24L01_0A_RX_ADDR_P0, (uint8_t*)(&radioPipeID), 5);
 NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (uint8_t*)(&radioPipeID_tele), 5);
 NRF24L01_WriteReg(NRF24L01_11_RX_PW_P0, 0x20); // 32 byte RX packet length
//NRF24L01_WriteRegisterMulti(NRF24L01_0B_RX_ADDR_P1, (uint8_t*)(&radioPipeID), 5); Ingwie : We don't use pipe1 here
//NRF24L01_WriteReg(NRF24L01_12_RX_PW_P1, 0x20); // 32 byte packet length

 NRF24L01_Activate(0x73);
 NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x3F); // Enable dynamic payload length on all pipes
 NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x04); // Enable dynamic Payload Length
 NRF24L01_Activate(0x73);
 NRF24L01_FlushTx();
 NRF24L01_FlushRx();
 pin_low(RADIO_CE_PIN);
}

uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
