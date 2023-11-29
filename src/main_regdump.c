//==============================================================================
//
//  main_regdump.c - Epson IMU sensor test application
//                 - This program reads all registers values for debug purpose
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
#include <string.h>
#include <time.h>

#ifdef SPI
#else
#include <termios.h>
#endif  // SPI

#include "hcl.h"
#include "hcl_gpio.h"

#ifdef SPI
#include "hcl_spi.h"
#else
#include "hcl_uart.h"
#endif  // SPI

#include "sensor_epsonCommon.h"

#ifdef SPI
#else

// Modify below as needed for hardware
const char* IMUSERIAL = "/dev/ttyUSB0";
#endif  // SPI

int main(int argc, char* argv[]) {
  char prod_id[9];  // Device Product ID
  char ser_id[9];   // Device Serial ID

  // 1) Initialize the Seiko Epson HCL layer
  printf("\r\nInitializing HCL layer...");
  if (!seInit()) {
    printf(
        "\r\nError: could not initialize the Seiko Epson HCL layer. "
        "Exiting...\r\n");
    return -1;
  }
  printf("...done.\r\n");

  // 2) Initialize the GPIO interfaces, For GPIO control of pins SPI CS, RESET,
  // DRDY
  printf("\r\nInitializing GPIO interface...");
  if (!gpioInit()) {
    printf("\r\nError: could not initialize the GPIO layer. Exiting...\r\n");
    seRelease();
    return -1;
  }
  printf("...done.\r\n");

#ifdef SPI
  // 3) Initialize SPI Interface
  printf("\r\nInitializing SPI interface...");
  // The max SPI clock rate is 1MHz for current model Epson IMUs
  if (!spiInit(SPI_MODE3, 500000)) {
    printf("\r\nError: could not initialize SPI interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
  sensorDummyWrite();
#else
  // 3) Initialize UART Interface
  //    The baudrate value should be set the the same setting as currently
  //    flashed value in the IMU UART_CTRL BAUD_RATE register
  printf("\r\nInitializing UART interface...");
  if (!uartInit(IMUSERIAL, BAUD_460800)) {
    printf("\r\nError: could not initialize UART interface. Exiting...\r\n");
    gpioRelease();
    seRelease();
    return -1;
  }
#endif  // SPI

  printf("...done.\r\n");

  // 4) Print out which model executable was compiled and identify model
  printf("\r\nCompiled for:\t" BUILD_FOR);
  printf("\r\nReading device info...");
  if (strcmp(BUILD_FOR, getProductId(prod_id)) != 0) {
    printf("\r\n*** Build *mismatch* with detected device ***");
    printf(
        "\r\n*** Ensure you specify a compatible 'MODEL=' variable when "
        "running make when  rebuilding the driver ***\r\n");
  }
  printf("\r\nPRODUCT ID:\t%s", getProductId(prod_id));
  printf("\r\nSERIAL ID:\t%s", getSerialId(ser_id));

  // Incase, the IMU is currently in sampling mode, force config mode before
  // attempting to read from registers
  sensorStop();
  registerDump();

#ifdef SPI
  spiRelease();
#else
  uartRelease();
#endif
  gpioRelease();
  seRelease();
  printf("\r\n");

  return 0;
}
