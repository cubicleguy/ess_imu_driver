//==============================================================================
//
//  hcl_spi.h - Seiko Epson Hardware Control Library
//
//  This layer of indirection is added to allow the sample code to call
// generic spi functions to work on multiple hardware platforms.
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

typedef enum {
  SPI_MODE0 = 0, /*!< CPOL = 0, CPHA = 0 */
  SPI_MODE1 = 1, /*!< CPOL = 0, CPHA = 1 */
  SPI_MODE2 = 2, /*!< CPOL = 1, CPHA = 0 */
  SPI_MODE3 = 3  /*!< CPOL = 1, CPHA = 1 */
} SPIMode;

#define SPI_CHAN 0

#ifdef __cplusplus
extern "C" {
#endif

// Prototypes for generic SPI functions
int spiInit(uint8_t mode, uint32_t khzspeed);
int spiRelease(void);
uint8_t spiTransfer(uint8_t value);

#ifdef __cplusplus
}
#endif
