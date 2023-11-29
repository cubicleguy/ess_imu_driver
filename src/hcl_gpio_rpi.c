//==============================================================================
//
//  hcl_gpio_rpi.c - Seiko Epson Hardware Control Library
//
//  This layer of indirection is added to allow the sample code to call
//  generic  functions to work on multiple hardware platforms, this is the
//  Raspberry Pi specific implementation for GPIO
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
#include <wiringPi.h>

#include "hcl.h"
#include "hcl_gpio.h"

/*****************************************************************************
** Function name:       gpioInit
** Description:         Initialize the RPI GPIO Interface.
** Parameters:          None
** Return value:        OK or NG
** Note:                This function assumes seInit() has been called first to
**                      initialise the wiringPI Library
*****************************************************************************/
int gpioInit(void) {
  pinMode(EPSON_RESET, OUTPUT);
  pinMode(EPSON_CS, OUTPUT);
  pinMode(EPSON_DRDY, INPUT);
  pullUpDnControl(EPSON_DRDY, PUD_OFF);

  return OK;
}

/*****************************************************************************
** Function name:       gpioRelease
** Description:         Release the RPI GPIO Interface.
** Parameters:          None
** Return value:        OK
*****************************************************************************/
int gpioRelease(void) { return OK; }

/*****************************************************************************
** Function name:       gpioSet
** Description:         Set the RPI GPIO pin level HIGH.
** Parameters:          uint8_t pin
** Return value:        None
*****************************************************************************/
void gpioSet(uint8_t pin) { digitalWrite(pin, HIGH); }

/*****************************************************************************
** Function name:       gpioClr
** Description:         Set the RPI GPIO pin level LOW.
** Parameters:          uint8_t pin
** Return value:        None
*****************************************************************************/
void gpioClr(uint8_t pin) { digitalWrite(pin, LOW); }

/*****************************************************************************
** Function name:       gpioGetPinLevel
** Description:         Get the RPI GPIO pin level.
** Parameters:          uint8_t pin
** Return value:        uint8_t level (1 or 0)
*****************************************************************************/
uint8_t gpioGetPinLevel(uint8_t pin) { return (digitalRead(pin)); }
