//==============================================================================
//
//  hcl_linux.c - Seiko Epson Hardware Control Library
//
//  This layer of indirection is added to allow the sample code to call
//  generic functions to work on linux-based hardware platforms.
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

#include "hcl.h"

/*****************************************************************************
** Function name:       seInit
** Description:         Initialize the Seiko Epson HCL libraries.
** Parameters:          None
** Return value:        1=OK
*****************************************************************************/
int seInit(void) { return OK; }

/*****************************************************************************
** Function name:       seRelease
** Description:         Release any resources held by this module.
** Parameters:          None
** Return value:        OK
*****************************************************************************/
int seRelease(void) { return OK; }

/*****************************************************************************
** Function name:       seDelayMS
** Description:         Call this function to generate delay in milliseconds.
** Parameters:          Time in milliseconds
** Return value:        None
*****************************************************************************/
void seDelayMS(uint32_t millis) {
  millis *= 1000;
  usleep(millis);
}

/*****************************************************************************
** Function name:       seDelayMicroSecs
** Description:         Call this function to generate delay in microseconds.
** Parameters:          Time in milliseconds
** Return value:        None
*****************************************************************************/
void seDelayMicroSecs(uint32_t micros) { usleep(micros); }
