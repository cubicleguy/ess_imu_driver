//==============================================================================
//
//	hcl_gpio.h - Seiko Epson Hardware Control Library
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

#include <stdint.h>

#if RPI
// Refer to https://github.com/WiringPi/pins for pin number and mapping
#define RPI_GPIO_P1_15 22
#define RPI_GPIO_P1_16 23
#define RPI_GPIO_P1_18 24

#define EPSON_RESET RPI_GPIO_P1_15
#define EPSON_CS RPI_GPIO_P1_16
#define EPSON_DRDY RPI_GPIO_P1_18

#else  // PC
// Dummy assignments if below signal are not connected to anything
#define EPSON_RESET 0
#define EPSON_DRDY 0
#define EPSON_CS 0
#endif  // RPI

#ifdef __cplusplus
extern "C" {
#endif

// Prototypes for generic GPIO functions
int gpioInit(void);
int gpioRelease(void);

void gpioSet(uint8_t pin);
void gpioClr(uint8_t pin);
uint8_t gpioGetPinLevel(uint8_t pin);

#ifdef __cplusplus
}
#endif
