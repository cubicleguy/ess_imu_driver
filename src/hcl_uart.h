//==============================================================================
//
//  hcl_uart.h - Seiko Epson Hardware Control Library
//
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

// Common supported BAUD rates.
// Only G330PDG0/G365PDF1/G366PDG0/G370PDF1/G370PDS0/G370PDG0/G370PDT0 can
// support 921600 baud. 921600 baud may not be supported on all host hardware
// platforms.
#define BAUD_921600 921600
#define BAUD_460800 460800
#define BAUD_230400 230400

#ifdef __cplusplus
extern "C" {
#endif

// Prototypes for generic UART functions
int uartInit(const char* comPortPath, int baudrate);
int uartRelease(void);
int readComPort(unsigned char* bytesToRead, int size);
int writeComPort(unsigned char* bytesToWrite, int size);
int numBytesReadComPort(void);
int purgeComPort(void);

#ifdef __cplusplus
}
#endif
