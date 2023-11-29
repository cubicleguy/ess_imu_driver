//==============================================================================
//
// 	sensor_epsonV340.h - Epson V340 sensor specific definitions
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

#define EPSON_ACCL_SF (.180)
#define EPSON_GYRO_SF (.015)
#define EPSON_TEMP_SF (-0.0053964)
#define EPSON_ATTI_SF (0.0)
#define EPSON_COUNT_SF (21333)

#define EPSON_DA_SF0 (0.0)
#define EPSON_DA_SF1 (0.0)
#define EPSON_DA_SF2 (0.0)
#define EPSON_DA_SF3 (0.0)
#define EPSON_DA_SF4 (0.0)
#define EPSON_DA_SF5 (0.0)
#define EPSON_DA_SF6 (0.0)
#define EPSON_DA_SF7 (0.0)
#define EPSON_DA_SF8 (0.0)
#define EPSON_DA_SF9 (0.0)
#define EPSON_DA_SF10 (0.0)
#define EPSON_DA_SF11 (0.0)
#define EPSON_DA_SF12 (0.0)
#define EPSON_DA_SF13 (0.0)
#define EPSON_DA_SF14 (0.0)
#define EPSON_DA_SF15 (0.0)

#define EPSON_DV_SF0 (0.0)
#define EPSON_DV_SF1 (0.0)
#define EPSON_DV_SF2 (0.0)
#define EPSON_DV_SF3 (0.0)
#define EPSON_DV_SF4 (0.0)
#define EPSON_DV_SF5 (0.0)
#define EPSON_DV_SF6 (0.0)
#define EPSON_DV_SF7 (0.0)
#define EPSON_DV_SF8 (0.0)
#define EPSON_DV_SF9 (0.0)
#define EPSON_DV_SF10 (0.0)
#define EPSON_DV_SF11 (0.0)
#define EPSON_DV_SF12 (0.0)
#define EPSON_DV_SF13 (0.0)
#define EPSON_DV_SF14 (0.0)
#define EPSON_DV_SF15 (0.0)

/*                   -- Commands --
- ADDR_ address byte of transfer to select the register to access
- CMD_  data byte of transfer to write to the register selected

- All accesses are 16 bit transfers
- For SPI IF:
        - For SPI write accesses - 8-bit address with msb=1b (can be even or
                                                           odd) + 8-bit write
data
                                                         - No response
        - For SPI read accesses  - 8-bit address with msb=0b(even only) + 8-bit
                                                           dummy data
                                                         - Response is
transferred on MOSI on next SPI access
                                                         - Return value is
16-bit read data (high byte + low byte)
- For UART IF:
        - For UART write accesses - 8-bit address with msb=1b(can be even or
                                odd) + 8-bit write data + Delimiter Byte
                                                          - No response
        - For UART read accesses  - 8-bit address with msb=0b(even only) + 8-bit
                                dummy data + Delimiter Byte
                                                          - Response is
transferred immediately
                                                          - Return value
consists of Register Read Address + 16-bit read data (high byte + low byte) +
Delimiter Byte

- NOTE: Register Address Maps that depend on the WINDOW_ID (page) */

// WINDOW_ID 0 (This is for compatibility with other IMUS, WINDOW_ID is not used
// by V340)
#define ADDR_FLAG 0x00         // FLAG(ND/EA) (W0)
#define ADDR_TEMP_LOW 0x02     // TEMPC Byte0 (W0)
#define ADDR_TEMP_HIGH 0x02    // TEMPC Byte1 (W0)
#define ADDR_XGYRO_HIGH 0x04   // XGYRO Byte0 (W0)
#define ADDR_XGYRO_LOW 0x04    // XGYRO Byte1 (W0)
#define ADDR_YGYRO_HIGH 0x06   // YGYRO Byte0 (W0)
#define ADDR_YGYRO_LOW 0x06    // YGYRO Byte1 (W0)
#define ADDR_ZGYRO_HIGH 0x08   // ZGYRO Byte0 (W0)
#define ADDR_ZGYRO_LOW 0x08    // ZGYRO Byte1 (W0)
#define ADDR_XACCL_HIGH 0x0A   // XACCL Byte0 (W0)
#define ADDR_XACCL_LOW 0x0A    // XACCL Byte1 (W0)
#define ADDR_YACCL_HIGH 0x0C   // YACCL Byte0 (W0)
#define ADDR_YACCL_LOW 0x0C    // YACCL Byte1 (W0)
#define ADDR_ZACCL_HIGH 0x0E   // ZACCL Byte0 (W0)
#define ADDR_ZACCL_LOW 0x0E    // ZACCL Byte1 (W0)
#define ADDR_GPIO 0x10         // GPIO  (W0)
#define ADDR_COUNT 0x12        // COUNT (W0)
#define ADDR_SIG_CTRL_LO 0x32  // SIG_CTRL Byte0 (W0)
#define ADDR_POL_CTRL_LO \
  0x32  // This is mirror of SIG_CTRL Byte0 (W0) for compatibility for other IMU
        // models
#define ADDR_SIG_CTRL_HI 0x33     // SIG_CTRL Byte1 (W0)
#define ADDR_MSC_CTRL_LO 0x34     // MSC_CTRL Byte0 (W0)
#define ADDR_MSC_CTRL_HI 0x35     // MSC_CTRL Byte1 (W0)
#define ADDR_SMPL_CTRL_LO 0x36    // SMPL_CTRL Byte0 (W0)
#define ADDR_SMPL_CTRL_HI 0x37    // SMPL_CTRL Byte1 (W0)
#define ADDR_FILTER_CTRL_LO 0x38  // FILTER_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_LO 0x38    // MODE_CTRL Byte0 (W0)
#define ADDR_MODE_CTRL_HI 0x39    // MODE_CTRL Byte1 (W0)
#define ADDR_UART_CTRL_LO 0x3A    // UART_CTRL Byte0 (W0)
#define ADDR_UART_CTRL_HI 0x3B    // UART_CTRL Byte1 (W0)
#define ADDR_DIAG_STAT 0x3C       // DIAG_STAT Byte0 (W0)
#define ADDR_GLOB_CMD_LO 0x3E     // GLOB_CMD Byte0 (W0)
#define ADDR_GLOB_CMD_HI 0x3F     // GLOB_CMD Byte1 (W0)
#define ADDR_COUNT_CTRL_LO 0x50   // COUNT_CTRL Byte0 (W0)

#define ADDR_PROD_ID1 0x6A     // PROD_ID1(W0)
#define ADDR_PROD_ID2 0x6C     // PROD_ID2(W0)
#define ADDR_PROD_ID3 0x6E     // PROD_ID3(W0)
#define ADDR_PROD_ID4 0x70     // PROD_ID4(W0)
#define ADDR_VERSION 0x72      // VERSION(W0)
#define ADDR_SERIAL_NUM1 0x74  // SERIAL_NUM1(W0)
#define ADDR_SERIAL_NUM2 0x76  // SERIAL_NUM2(W0)
#define ADDR_SERIAL_NUM3 0x78  // SERIAL_NUM3(W0)
#define ADDR_SERIAL_NUM4 0x7A  // SERIAL_NUM4(W0)

#define ADDR_WIN_CTRL 0x7E  // This is for compatibility for other IMU models

#define CMD_BURST 0x20    // BURST
#define CMD_WINDOW0 0x00  // Write value for WIN_CTRL to change to Window 0
#define CMD_WINDOW1 0x01  // Write value for WIN_CTRL to change to Window 1
#define CMD_BEGIN_SAMPLING \
  0x01                         // Write value for MODE_CMD_HI to begin sampling
#define CMD_END_SAMPLING 0x02  // Write value for MODE_CMD_HI to stop sampling
#define CMD_SOFTRESET \
  0x80  // Write value for GLOB_CMD_LO to issue Software Reset
#define CMD_FLASHTEST 0x08  // Write value for MSC_CTRL_HI to issue Flashtest
#define CMD_SELFTEST 0x04   // Write value for MSC_CTRL_HI to issue Selftest

// Write values for ADDR_SMPL_CTRL_HI to set Output Rate
#define CMD_RATE1000 0x01   // TAP>=1
#define CMD_RATE500 0x02    // TAP>=2
#define CMD_RATE250 0x03    // TAP>=4
#define CMD_RATE125 0x04    // TAP>=8
#define CMD_RATE62_5 0x05   // TAP>=16
#define CMD_RATE31_25 0x06  // TAP>=32

// Write values for FILTER_CTRL_LO to set Filter
#define CMD_FLTAP1 0x02
#define CMD_FLTAP2 0x03
#define CMD_FLTAP4 0x04
#define CMD_FLTAP8 0x05
#define CMD_FLTAP16 0x06
#define CMD_FLTAP32 0x07

// MODE STAT
#define VAL_SAMPLING_MODE 0x00
#define VAL_CONFIG_MODE 0x04
