//==============================================================================
//
//	hcl_gpio.c - Seiko Epson Hardware Control Library
//
//	This layer of indirection is added to allow the sample code to call
//  generic functions to work on multiple hardware platforms, this is generic
//	implementation for GPIO function which may be needed for optionally
//  connecting pins RESET#, SPI Chipselect, DataReady, EXT of the IMU
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

#include "hcl.h"
#include "hcl_gpio.h"

/*****************************************************************************
** Function name:       gpioInit
** Description:         Initialize generic GPIO library (if any)
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int gpioInit(void) { return OK; }

/*****************************************************************************
** Function name:       gpioInit
** Description:         Release generic GPIO interface (if any)
** Parameters:          None
** Return value:        OK
*****************************************************************************/
int gpioRelease(void) { return OK; }

/*****************************************************************************
** Function name:       gpioSet
** Description:         Generic GPIO pin set to HIGH
** Parameters:          Pin number
** Return value:        None
*****************************************************************************/
void gpioSet(__attribute__((unused)) uint8_t pin) { return; }

/*****************************************************************************
** Function name:       gpioClr
** Description:         Generic GPIO pin set to LOW
** Parameters:          Pin number
** Return value:        None
*****************************************************************************/
void gpioClr(__attribute__((unused)) uint8_t pin) { return; }

/*****************************************************************************
** Function name:       gpioGetPinLevel
** Description:         Generic read GPIO pin status
** Parameters:          Pin number
** Return value:        1 = GPIO pin HIGH, 0 = GPIO pin LOW
*****************************************************************************/
uint8_t gpioGetPinLevel(__attribute__((unused)) uint8_t pin) { return 0; }
