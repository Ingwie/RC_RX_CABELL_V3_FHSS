/*         Copyright 2020 by Ingwie (Bracame)          */
/*    This is a fork modded of RC_RX_CABELL_V3_FHSS    */
/* https://github.com/soligen2010/RC_RX_CABELL_V3_FHSS */
/*   Licence: GPLV3 see <http://www.gnu.org/licenses   */
/*        Compile with AVR GCC + Code::Blocks          */
/*    https://www.mediafire.com/file/cahqfrm90h7c7fy/  */
/*    Setup_OAVRCBuilder3.exe/file (Pswd : OpenAVRc)   */
/*       Old header if it is not a new file ->         */


#include "spi.h"
//---------------------------
// AVR SPI functions
//---------------------------

void master_spi_disable()
{
 SPCR &= ~_BV(SPE);
}

//----------------------

void enable_spi_master_mode()
{
// Enable SPI as Master, MSB first.
#define SPI_4M()                    { SPCR = _BV(SPE) | _BV(MSTR); SPSR = 0x00; }
#define SPI_8M()                    { SPCR = _BV(SPE) | _BV(MSTR); SPSR |= _BV(SPI2X); }
 SPI_4M(); // Ingwie Todo switch to 8M
}

//----------------------

uint8_t master_spi_xfer(uint8_t value)
{
// Full Duplex (4 wire) spi
 SPDR = value;
 /* Wait for transfer to complete */
 while (!(SPSR & _BV(SPIF)));
 return SPDR;
}
