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

/* Library dependencies:
 *
 *  Aaduino Core, SPI, and EEPROM
 *
 * https://github.com/nRF24/RF24 library copied and modified to streamline
 * opeerations specific to this application in order to improve loop timing.
 * See http://tmrh20.github.io/RF24  for documentation on the standard version of the library.
 *
 * Arduino Servo was modified and is included with this source.  It was changed to not directly define the Timer 1 ISR
 */

#include "RX.h"
#include "Pins.h"
#include "PPM.h"

//--------------------------------------------------------------------------------------------------------------------------
void setupAll()
{
 /* Init pin */
// UART0
 set_output_on(UART_TX_PIN);
 set_input_pullup(UART_RX_PIN);
// SPI
 set_input_pullup(SPI_MISO_PIN);
 set_output_on(SPI_MOSI_PIN);
 set_output_on(SPI_SCLK_PIN);
 set_output_on(RADIO_CSN_PIN);
 set_output_off(RADIO_CE_PIN);
// I2C (actually no used)
 set_input_pullup(I2C_SDA_PIN);
 set_input_pullup(I2C_SCL_PIN);

 set_input_pullup(RADIO_IRQ_PIN);

 set_input_pullup(BIND_BUTTON_PIN);  // used for bind plug or button

 set_output_off(LED_PIN);            //status LED

 set_input_pullup(PPM_OUTPUT_PIN);   // Set this pin mode on PPM pin to keep PPM from floating until the output mode is received.  A 10k pull-up resistor is better as the pin floats until this line runs
 PPMDisable();                       // Sets input pullup on all servo pins to they dont float.

//Serial.begin(74880);
//Serial.println(); //Serial.println(F("Initializing"));

 init_time_machine(); // Timer2 used to measure time

 enable_spi_master_mode(); // Init SPI (master mode)

 ADC_Processing();   // Initial analog reads for A6/A7.  Initial call returns bad value so call 3 times to get a good starting value from each pin
 while (!bit_is_clear(ADCSRA, ADSC));  // wait for conversion
 ADC_Processing();
 while (!bit_is_clear(ADCSRA, ADSC));  // wait for conversion
 ADC_Processing();

 setupReciever();

//Serial.println(F("Starting main loop "));
}

//--------------------------------------------------------------------------------------------------------------------------
int main()
{
 setupAll();

 do
  {
   if (getPacket())
    {
     outputChannels();
    }
   ADC_Processing();   // Process ADC to asynchronously read A6 and A7 for telemetry analog values.  Non-blocking read
  }
 while (true);        //loop forever
}





