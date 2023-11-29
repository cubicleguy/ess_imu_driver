//==============================================================================
//
//  hcl_uart.c - Seiko Epson Hardware Control Library
//
//  This layer of indirection is added to allow the sample code to call generic
//  UART functions to work on multiple hardware platforms. This is the Linux
//  TERMIOS specific implementation.
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

#include <assert.h>
#include <errno.h>  // Error number definitions
#include <fcntl.h>  // File control definitions
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>  // Needed for ioctl library functions
#include <termios.h>    // terminal io (serial port) interface
#include <unistd.h>

#include "hcl.h"
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"

extern const char* IMUSERIAL;
static int fd_serial;

typedef unsigned char Byte;

// Local function prototypes
int openComPort(const char* comPortPath, speed_t baudRate);

void closeComPort(void);

/*****************************************************************************
** Function name:       uartInit
** Description:         Initialize the COM port with the settings for
**                      communicating with the connected Epson IMU.
**                      Call this function instead of openComPort().
** Parameters:          Pointer to device name, baudrate
** Return value:        OK or NG
*****************************************************************************/
int uartInit(const char* comPortPath, int baudrate) {
  printf("Attempting to open port...%s", comPortPath);

  speed_t baudRate;

  switch (baudrate) {
    case BAUD_921600:
      baudRate = B921600;
      break;
    case BAUD_460800:
      baudRate = B460800;
      break;
    case BAUD_230400:
      baudRate = B230400;
      break;
    default:
      printf("Invalid baudrate\n");
      return NG;
  }
  return openComPort(comPortPath, baudRate);
}

/*****************************************************************************
** Function name:       uartRelease
** Description:         Release the COM port (close) after a 100msec delay
**                      and closing the com port to the Epson IMU.
**                      Call this function instead of closeComPort().
** Parameters:          COM port handle
** Return value:        OK
*****************************************************************************/
int uartRelease(void) {
  seDelayMicroSecs(
      100000);  // Provide 100msec delay for any pending transfer to complete
  closeComPort();
  return OK;
}

/*****************************************************************************
** Function name:       readComPort
** Description:         Read the specified number of bytes from the COM port
** Parameters:          COM port handle, pointer to output char array, # of
**                      bytes to read
** Return value:        # of bytes returned by COM port, or -1=NG
*****************************************************************************/
int readComPort(unsigned char* bytesToRead, int size) {
  return read(fd_serial, bytesToRead, size);
}

/*****************************************************************************
** Function name:       writeComPort
** Description:         Write specified number of bytes to the COM port
** Parameters:          COM port handle, pointer to input char array, # of bytes
**                      to send
** Return value:        # of bytes sent, or -1=NG
*****************************************************************************/
int writeComPort(unsigned char* bytesToWrite, int size) {
  return write(fd_serial, bytesToWrite, size);
}

/*****************************************************************************
** Function name:       numBytesReadComPort
** Description:         Returns number of bytes in COM port read buffer
**                      Purpose is to check if data is available
** Parameters:          COM port handle
** Return value:        # of bytes in the COM port receive buffer
*****************************************************************************/
int numBytesReadComPort(void) {
  int numBytes;

  ioctl(fd_serial, FIONREAD, &numBytes);
  return numBytes;
}

/*****************************************************************************
** Function name:       purgeComPort
** Description:         Clears the com port's read and write buffers
** Parameters:          COM port handle
** Return value:        OK or NG
*****************************************************************************/
int purgeComPort(void) {
  if (tcflush(fd_serial, TCIOFLUSH) == -1) {
    printf("flush failed\n");
    return NG;
  }
  return OK;
}

/*****************************************************************************
** Function name:       openComPort
** Description:         Com port is opened in raw mode and
**                      will timeout on reads after 2 second.
**                      This will return a NG if the port could not open or
**                      the port options could not be set.
**                      This is not intended to be called directly, but is
**                      called from uartInit()
** Parameters:          Pointer to device name, Baudrate
** Return value:        OK or NG
*****************************************************************************/
int openComPort(const char* comPortPath, speed_t baudRate) {
  // Read/Write, Not Controlling Terminal
  fd_serial = open(comPortPath, O_RDWR | O_NOCTTY);

  if (fd_serial < 0)  // Opening of port is NG
  {
    printf("Unable to open com Port %s\n", comPortPath);
    return NG;
  }

  // Get the current options for the port...
  struct termios options;
  tcgetattr(fd_serial, &options);

  // Set the baud rate to 460800
  cfsetospeed(&options, baudRate);
  cfsetispeed(&options, baudRate);

  // Turn off character processing
  // Clear current char size mask
  // Force 8 bit input
  options.c_cflag &= ~CSIZE;  // Mask the character size bits
  options.c_cflag |= CS8;

  // Set the number of stop bits to 1
  options.c_cflag &= ~CSTOPB;

  // Set parity to None
  options.c_cflag &= ~PARENB;

  // Set for no input processing
  options.c_iflag = 0;

  // From https://en.wikibooks.org/wiki/Serial_Programming/termios
  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control
  //    options.c_iflag &= ~(IGNPAR | IGNBRK | BRKINT | IGNCR | ICRNL |
  //        INLCR | PARMRK | INPCK | ISTRIP | IXON | IXOFF | IXANY);

  // Output flags - Turn off output processing

  // From http://www.cmrr.umn.edu/~strupp/serial.html
  // options.c_oflag &= ~OPOST;
  options.c_oflag = 0;  // raw output

  // No line processing
  // echo off, echo newline off, canonical mode off,
  // extended input processing off, signal chars off

  // From http://www.cmrr.umn.edu/~strupp/serial.html
  // options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);options.c_lflag &=
  // ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_lflag = 0;  // raw input

  // From http://www.cmrr.umn.edu/~strupp/serial.html
  // Timeouts are ignored in canonical input mode or when the NDELAY option is
  // set on the file via open or fcntl. VMIN specifies the minimum number of
  // characters to read. 1) If VMIN is set to 0, then the VTIME value specifies
  // the time to wait for every characters to be read. The read call will return
  // even if less than specified from the read request. 2) If VMIN is non-zero,
  // VTIME specifies the time to wait for the first character. If first
  // character is received within the specified VTIME, then it won't return
  // until VMIN number of characters are received. So any read call can return
  // either 0 characters or N-specified characters, but nothing inbetween. It
  // will block forever if RX characters are not in multiples of VMIN. 3) VTIME
  // specifies the amount of time to wait for incoming characters in tenths of
  // seconds. If VTIME is set to 0 (the default), reads will block (wait)
  // indefinitely unless the NDELAY option is set on the port with open or fcntl
  // Setting VTIME = 0, makes UART reads blocking, try experimenting with value
  // to prevent hanging waiting for reads

  // Current setting below: Non-blocking reads with first character recv timeout
  // of 2 seconds
  options.c_cc[VMIN] = 4;  // block reading until VMIN 4 of characters are read.
  options.c_cc[VTIME] =
      20;  // Inter-Character Timer -- i.e. timeout= x*.1 s = 2 seconds

  // Set local mode and enable the receiver
  options.c_cflag |= (CLOCAL | CREAD);

  // Set the new options for the port...
  int status = tcsetattr(fd_serial, TCSANOW, &options);

  if (status != 0)  // For error message
  {
    printf("Configuring comport failed\n");
    closeComPort();
    return NG;
  }

  // Purge serial port buffers
  purgeComPort();

  return OK;
}

/*****************************************************************************
** Function name:       closeComPort
** Description:         Closes a Com port (previously opened with OpenComPort)
**                      This is not intended to be called directly, but is
**                      called from uartRelease()
** Parameters:          COM port handle
** Return value:        None
*****************************************************************************/
void closeComPort(void) { close(fd_serial); }
