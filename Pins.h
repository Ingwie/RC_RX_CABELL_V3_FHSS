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

#ifndef __have__RC_RX_PINS_h__
#define __have__RC_RX_PINS_h__

#include "pin_helper.h"
#include "RX.h"

#define RADIO_CSN_PIN            C,0 // PC0
#define RADIO_CE_PIN             B,2 // PB2 Set CE pin to CSN puts radio in a mode where CE is not used (always pulled high)
#define RADIO_IRQ_PIN            C,2 // PC2 configure A2 for radio IRQ

#define SPI_MOSI_PIN             B,3  // PB3
#define SPI_MISO_PIN             B,4  // PB4
#define SPI_SCLK_PIN             B,5  // PB5

#define I2C_SDA_PIN              C,4  //PC4
#define I2C_SCL_PIN              C,5  //PC5

#define ROLL_PIN                 D,2  // PD2
#define PITCH_PIN                D,3  // PD3
#define THROTTLE_PIN             D,4  // PD4
#define YAW_PIN                  D,5  // PD5
#define AUX1_PIN                 D,6  // PD6
#define AUX2_PIN                 D,7  // PD7
#define AUX3_PIN                 B,0  // PB0
#define AUX4_PIN                 B,1  // PB1

#define PPM_OUTPUT_PIN           ROLL_PIN // PD2

#define BIND_BUTTON_PIN          C,3 // PC3
#define LED_PIN                  C,1 // PC1

#define UART_TX_PIN              D,0 // PD0
#define UART_RX_PIN              D,1 // PD1

#define TELEMETRY_ANALOG_INPUT_1   6 // Pin ADC6
#define TELEMETRY_ANALOG_INPUT_2   7 // Pin ADC7

#ifdef TEST_HARNESS
 // Pins for test harness LCD display
 #define D4_PIN     9
 #define D5_PIN     8
 #define D6_PIN     7
 #define D7_PIN     6
 #define RS_PIN     5
 #define EN_PIN     4
#endif

#endif
