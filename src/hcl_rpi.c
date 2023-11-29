//==============================================================================
//
//  hcl_rpi.c - Seiko Epson Hardware Control Library
//
//  This layer of indirection is added to allow the sample code to call
//  generic functions to work on multiple hardware platforms. This is the
//  Raspberry Pi specific implementation which uses the wiringPi library,
//  whose init needs to be called.
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
#include <wiringPi.h>
#include "hcl.h"

/*****************************************************************************
** Function name:       seInit
** Description:         Initialize the Seiko Epson Hardware Control Library
**                      In this case, we initialize the wiringPi library.
** Parameters:          None
** Return value:        OK if successful, otherwise NG
*****************************************************************************/
int seInit(void) {
  // Initialize wiringPi libraries
  printf("\r\nInitializing libraries...");
  if (wiringPiSetupGpio() != 0) {
    printf(
        "\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");
    return NG;
  }
  printf("...done.");

  return OK;
}

/*****************************************************************************
** Function name:       seRelease
** Description:         Release any resources held by this module.
** Parameters:          None
** Return value:        OK
*****************************************************************************/
int seRelease(void) { return OK; }

/*****************************************************************************
** Function name:       seDelayMS
** Description:         Generates software delay in milliseconds.
** Parameters:          Delay value in milliseconds
** Return value:        None
*****************************************************************************/
void seDelayMS(uint32_t millis) { delay(millis); }

/*****************************************************************************
** Function name:       seDelayMicroSecs
** Description:         Generates software delay in microseconds.
** Parameters:          Delay value in microseconds
** Return value:        None
*****************************************************************************/
void seDelayMicroSecs(uint32_t micros) { delayMicroseconds(micros); }
