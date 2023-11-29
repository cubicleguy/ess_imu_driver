//==============================================================================
//
//  main_helper.h - Epson IMU helper functions headers for console utilities
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

#pragma once

#include "sensor_epsonCommon.h"

#ifdef __cplusplus
extern "C" {
#endif

// Prototypes
void printHeaderRow(FILE *fp, struct EpsonOptions);
void printSensorRow(FILE *fp, struct EpsonOptions, struct EpsonData *, int);

#ifdef __cplusplus
}
#endif
