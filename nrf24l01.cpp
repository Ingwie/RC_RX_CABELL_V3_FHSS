/*         Copyright 2020 by Ingwie (Bracame)          */
/*    This is a fork modded of RC_RX_CABELL_V3_FHSS    */
/* https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*        Compile with AVR GCC + Code::Blocks          */
/*    https://www.mediafire.com/file/cahqfrm90h7c7fy/  */
/*    Setup_OAVRCBuilder3.exe/file (Pswd : OpenAVRc)   */
/*       Old header if it is not a new file ->         */


#include "RX.h"
#include "spi.h"
#include "Pins.h"
#include "iface_nrf24l01.h"


uint8_t NRF24L01_WriteReg(uint8_t reg, uint8_t data)
{
 pin_low(RADIO_CSN_PIN);
 uint8_t res = master_spi_xfer(W_REGISTER | (REGISTER_MASK & reg));
 master_spi_xfer(data);
 pin_high(RADIO_CSN_PIN);
 return res;
}

uint8_t NRF24L01_WriteRegisterMulti(uint8_t reg, uint8_t data[], uint8_t length)
{
 pin_low(RADIO_CSN_PIN);
 uint8_t res = master_spi_xfer(W_REGISTER | ( REGISTER_MASK & reg));
 for (uint8_t i = 0; i < length; i++)
  {
   master_spi_xfer(data[i]);
  }
 pin_high(RADIO_CSN_PIN);
 return res;
}

uint8_t NRF24L01_WritePayload(uint8_t *data, uint8_t length)
{
 pin_low(RADIO_CSN_PIN);
 uint8_t res = master_spi_xfer(W_TX_PAYLOAD);
 for (uint8_t i = 0; i < length; i++)
  {
   master_spi_xfer(data[i]);
  }
 pin_high(RADIO_CSN_PIN);
 return res;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
 pin_low(RADIO_CSN_PIN);
 master_spi_xfer(R_REGISTER | (REGISTER_MASK & reg));
 uint8_t data = master_spi_xfer(0);
 pin_high(RADIO_CSN_PIN);
 return data;
}

uint8_t NRF24L01_ReadRegisterMulti(uint8_t reg, uint8_t data[], uint8_t length)
{
 pin_low(RADIO_CSN_PIN);
 uint8_t res = master_spi_xfer(R_REGISTER | (REGISTER_MASK & reg));
 for(uint8_t i = 0; i < length; i++)
  {
   data[i] = master_spi_xfer(0);
  }
 pin_high(RADIO_CSN_PIN);
 return res;
}

uint8_t NRF24L01_ReadPayload(uint8_t *data, uint8_t length)
{
 pin_low(RADIO_CSN_PIN);
 uint8_t res = master_spi_xfer(R_RX_PAYLOAD);
 for(uint8_t i = 0; i < length; i++)
  {
   data[i] = master_spi_xfer(0);
  }
 pin_high(RADIO_CSN_PIN);
 return res;
}

static uint8_t Strobe(uint8_t state)
{
 pin_low(RADIO_CSN_PIN);
 uint8_t res = master_spi_xfer(state);
 pin_high(RADIO_CSN_PIN);
 return res;
}

uint8_t NRF24L01_FlushTx()
{
 return Strobe(FLUSH_TX);
}

uint8_t NRF24L01_FlushRx()
{
 return Strobe(FLUSH_RX);
}

uint8_t NRF24L01_NOP()
{
 return Strobe(NOP);
}

uint8_t NRF24L01_Activate(uint8_t code)
{
 pin_low(RADIO_CSN_PIN);
 uint8_t res = master_spi_xfer(ACTIVATE);
 master_spi_xfer(code);
 pin_high(RADIO_CSN_PIN);
 return res;
}

uint8_t NRF24L01_SetBitrate(uint8_t bitrate)
{
 uint8_t temp = NRF24L01_ReadReg(NRF24L01_06_RF_SETUP);
 temp = (temp & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
 return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, temp);
}

uint8_t NRF24L01_SetPower(uint8_t nrf_power)
{
 /*
 // nRF24L01+ Power Output
      Raw       * 20dBm PA
 0 : -18dBm    2dBm (1.6mW)
 1 : -12dBm    8dBm   (6mW)
 2 :  -6dBm   14dBm  (25mW)
 3 :   0dBm   20dBm (100mW)
 */
 uint8_t temp = NRF24L01_ReadReg(NRF24L01_06_RF_SETUP);
 temp = (temp & 0xF9) | ((nrf_power & 0x03) << 1);
 return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, temp);
}
