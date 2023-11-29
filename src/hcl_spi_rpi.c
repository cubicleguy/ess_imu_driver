//==============================================================================
//
//  hcl_spi_rpi.c - Seiko Epson Hardware Control Library
//
//  This layer of indirection is for the Raspberry Pi specific implementation
//  of the SPI protocol. It uses the wiringPi library for the low-level SPI
//  transfers.
//
//
//  THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
//  PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
//  OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
//  SOFTWARE.
//
//==============================================================================

#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPiSPI.h>

#include "hcl.h"
#include "hcl_spi.h"
#include "sensor_epsonCommon.h"

static int mySpiFd;

/*****************************************************************************
** Function name:       spiInit
** Description:         Initialize the RPI spi library.
**
** NOTE: This function assumes that the seInit function has been called first
** and initialized the wiringPi library. Then the gpioInit called and setup all
** the necessary pins. CS should be HIGH in case of any glitches on the SPI bus
** during its setup
**
** Parameters:          SPI Mode, SPI clock speed
** Return value:        OK, or NG
*****************************************************************************/
int spiInit(uint8_t mode, uint32_t khzspeed) {
  if (mode != SPI_MODE3) {
    return NG;
  }

  if ((mySpiFd = wiringPiSPISetupMode(SPI_CHAN, khzspeed, mode)) < 0) {
    return NG;
  }

  return OK;
}

/*****************************************************************************
** Function name:       spiRelease
** Description:         Release the SPI Interface.
**
** Parameters:          None
** Return value:        OK
*****************************************************************************/
int spiRelease(void) {
  close(mySpiFd);
  return OK;
}

/*****************************************************************************
** Function name:       spiTransfer
** Description:         Initiates an 8-bit SPI transfer
**                      Transfers out 8-bit on MOSI
**                      Transfers in 8-bit on MISO
** Parameters:          8-bit Data to Send to SPI Slave
** Return value:        8-bit Data Received from SPI Slave
*****************************************************************************/
uint8_t spiTransfer(uint8_t value) {
  wiringPiSPIDataRW(SPI_CHAN, &value, 1);
  return value;
}
