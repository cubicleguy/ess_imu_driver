//==============================================================================
//
//  sensor_epsonUart.c - Epson IMU sensor protocol UART specific code
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

#include <stdio.h>

#include "hcl.h"
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"

// This is declared by the main() application for UART IF
extern const char* IMUSERIAL;  // COM port device name

// UART Interface Timing
// TWRITERATE/TREADRATE = 200us min @ 460800 BAUD, 1 command = 3 bytes = 3 *
// 22us = 66us TSTALL = 200us - 66us = 134us
#define EPSON_STALL 134  // Microseconds

// UART Byte Markers
#ifdef V340PDD0
const unsigned char UART_HEADER =
    0x20;  // Placed at the start of all UART cycles
#else
const unsigned char UART_HEADER =
    0x80;  // Placed at the start of all UART cycles
#endif
const unsigned char UART_DELIMITER =
    0x0D;  // Placed at the end of all UART cycles

static unsigned char rxByteBuf[256];  // COM port receive buffer

// Macros & variables used by state machine for processing UART burst read data
#define START 0
#define DATA 1
#define END 2
static int state = START;
static int data_count = 0;

// Flag and variables to store Delta Velocity & Delta Angle scale factors
static int scale_factors_initialized = 0;
static double dv_scale_factors[16];
static double da_scale_factors[16];

/*****************************************************************************
** Function name:       sensorDataReadyOptions
** Description:         For UART interface check if comport recv buffer
**                      contains a burst of data based on expected byte length
**                      from sensorDataByteLength()
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorDataReadyOptions(struct EpsonOptions options) {
  seDelayMicroSecs(100);
  unsigned int count = numBytesReadComPort();

  if (count >= sensorDataByteLength(options)) return OK;
  return NG;
}

/*****************************************************************************
** Function name:       registerWriteByteNoID
** Description:         Write Byte to Register = Write Data
**                      to Register (no WIN_ID)
**                      NOTE: V340 does not have WINDOW_ID function
**                            winNumber input parameter is ignored
** Parameters:          Register Address, Register Write Byte,
**                      Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByteNoId(unsigned char regAddr, unsigned char regByte,
                           unsigned int verbose) {
  unsigned char txData[3];

  txData[0] = regAddr | 0x80;  // msb is 1b for register writes
  txData[1] = regByte;
  txData[2] = UART_DELIMITER;
  writeComPort(txData, 3);
  EpsonStall();

  if (verbose) {
    printf("\r\nREG[0x%02X] < 0x%02X\t", regAddr, regByte);
  }
}

/*****************************************************************************
** Function name:       registerWriteByte
** Description:         Write Byte to Register = Set WIN_ID, Write Data
**                      to Register
**                      NOTE: V340 does not have WINDOW_ID function
**                            winNumber input parameter is ignored
** Parameters:          Window Number, Register Address, Register Write Byte,
**                      Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByte(unsigned char winNumber, unsigned char regAddr,
                       unsigned char regByte, unsigned int verbose) {
#if !(defined V340PDD0)
  unsigned char txData[3];

  txData[0] = ADDR_WIN_CTRL | 0x80;  // msb is 1b for register writes
  txData[1] = winNumber;
  txData[2] = UART_DELIMITER;
  writeComPort(txData, 3);
  EpsonStall();
#endif

  registerWriteByteNoId(regAddr, regByte, verbose);
}

/*****************************************************************************
** Function name:       registerRead16NoId
** Description:         Read 16-bit from Register (No WIN_ID)
** Parameters:          Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16NoId(unsigned char regAddr, unsigned int verbose) {
  unsigned char response[4] = {0};
  int size;
  unsigned char txData[3];

  txData[0] =
      regAddr & 0x7E;  // msb is 0b for register reads & address must be even
  txData[1] = 0x00;
  txData[2] = UART_DELIMITER;
  writeComPort(txData, 3);

  EpsonStall();

  // Attempt to read 4 bytes from serial port
  // Validation check: Should be atleast 4 bytes, First byte should be Register
  // Address,
  //                   Last byte should be delimiter
  size = readComPort(&response[0], 4);
  EpsonStall();

  if ((size < 4) || (response[0] != txData[0]) ||
      (response[3] != UART_DELIMITER)) {
    printf("Returned less data or unexpected data from previous command.\n");
    printf("Return data: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", response[0],
           response[1], response[2], response[3]);
  }

  if (verbose) {
    printf("REG[0x%02X] > 0x%02X%02X\t", regAddr, response[1], response[2]);
  }
  return (unsigned short)response[1] << 8 | (unsigned short)response[2];
}

/*****************************************************************************
** Function name:       registerRead16
** Description:         Read 16-bit from Register
**                      NOTE: V340 does not have WINDOW_ID function
**                            winNumber input parameter is ignored
** Parameters:          Window Number, Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16(unsigned char winNumber, unsigned char regAddr,
                              unsigned int verbose) {
#if !(defined V340PDD0)
  unsigned char txData[3];

  txData[0] = ADDR_WIN_CTRL | 0x80;
  txData[1] = winNumber;
  txData[2] = UART_DELIMITER;
  writeComPort(txData, 3);
  EpsonStall();
#endif

  return registerRead16NoId(regAddr, verbose);
}

/*****************************************************************************
** Function name:       populateEpsonData
** Description:         Loads Delta Angle/Velocity scale factors the one time
**                      Retrieves burst data buffer and converts/parses into
*struct
**                      based on configuration.
** Parameters:          options - struct describing IMU configuration.
**                      epson_data - struct that is filled with converted data.
** Return value:        none
** Notes:
******************************************************************************/
void populateEpsonData(struct EpsonOptions options,
                       struct EpsonData* epson_data) {
  // If not done so, store Delta Angle/Velocity scale factors
  if (!scale_factors_initialized) {
    da_scale_factors[0] = EPSON_DA_SF0;
    da_scale_factors[1] = EPSON_DA_SF1;
    da_scale_factors[2] = EPSON_DA_SF2;
    da_scale_factors[3] = EPSON_DA_SF3;
    da_scale_factors[4] = EPSON_DA_SF4;
    da_scale_factors[5] = EPSON_DA_SF5;
    da_scale_factors[6] = EPSON_DA_SF6;
    da_scale_factors[7] = EPSON_DA_SF7;
    da_scale_factors[8] = EPSON_DA_SF8;
    da_scale_factors[9] = EPSON_DA_SF9;
    da_scale_factors[10] = EPSON_DA_SF10;
    da_scale_factors[11] = EPSON_DA_SF11;
    da_scale_factors[12] = EPSON_DA_SF12;
    da_scale_factors[13] = EPSON_DA_SF13;
    da_scale_factors[14] = EPSON_DA_SF14;
    da_scale_factors[15] = EPSON_DA_SF15;

    dv_scale_factors[0] = EPSON_DV_SF0;
    dv_scale_factors[1] = EPSON_DV_SF1;
    dv_scale_factors[2] = EPSON_DV_SF2;
    dv_scale_factors[3] = EPSON_DV_SF3;
    dv_scale_factors[4] = EPSON_DV_SF4;
    dv_scale_factors[5] = EPSON_DV_SF5;
    dv_scale_factors[6] = EPSON_DV_SF6;
    dv_scale_factors[7] = EPSON_DV_SF7;
    dv_scale_factors[8] = EPSON_DV_SF8;
    dv_scale_factors[9] = EPSON_DV_SF9;
    dv_scale_factors[10] = EPSON_DV_SF10;
    dv_scale_factors[11] = EPSON_DV_SF11;
    dv_scale_factors[12] = EPSON_DV_SF12;
    dv_scale_factors[13] = EPSON_DV_SF13;
    dv_scale_factors[14] = EPSON_DV_SF14;
    dv_scale_factors[15] = EPSON_DV_SF15;
    scale_factors_initialized = 1;
  }

  // stores the sensor data array index when parsing out data fields
  int idx = 0;

#ifdef V340PDD0
  // Fixed packet format except for enabling/disabling count_out
  unsigned short ndflags = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
  epson_data->ndflags = ndflags;
  idx += 2;

  short temp = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
  epson_data->temperature = (temp - 2634) * EPSON_TEMP_SF + 25;
  idx += 2;

  short gyro_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
  short gyro_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
  short gyro_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
  epson_data->gyro_x = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_x;
  epson_data->gyro_y = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_y;
  epson_data->gyro_z = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_z;
  idx += 6;

  short accel_x = (rxByteBuf[idx] << 8) + (rxByteBuf[idx] + 1);
  short accel_y = (rxByteBuf[idx + 2] << 8) + (rxByteBuf[idx + 3]);
  short accel_z = (rxByteBuf[idx + 4] << 8) + (rxByteBuf[idx + 5]);
  epson_data->accel_x = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_x;
  epson_data->accel_y = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_y;
  epson_data->accel_z = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_z;
  idx += 6;
  unsigned short gpio = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
  idx += 2;
  epson_data->gpio = gpio;

#else
  // parsing of data fields applying conversion factor if applicable
  if (options.flag_out) {
    unsigned short ndflags = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    epson_data->ndflags = ndflags;
    idx += 2;
#ifdef DEBUG
    printf("ndflag: %04x\t", epson_data->ndflags);
#endif
  }

  if (options.temp_out) {
    if (options.temp_bit) {
      int temp = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
#if defined G330PDG0 || defined G366PDG0 || defined G370PDG0 || defined G370PDT0
      epson_data->temperature = (temp)*EPSON_TEMP_SF / 65536 + 25;
#else
      epson_data->temperature = (temp - 172621824) * EPSON_TEMP_SF / 65536 + 25;
#endif  // defined G330PDG0 || defined G366PDG0 || defined G370PDG0 || defined
        // G370PDT0
      idx += 4;
    } else {
      short temp = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
#if defined G330PDG0 || defined G366PDG0 || defined G370PDG0 || defined G370PDT0
      epson_data->temperature = (temp)*EPSON_TEMP_SF + 25;
#else
      epson_data->temperature = (temp - 2634) * EPSON_TEMP_SF + 25;
#endif  // defined G330PDG0 || defined G366PDG0 || defined G370PDG0 || defined
        // G370PDT0
      idx += 2;
    }
#ifdef DEBUG
    printf("tempC: %8.3f\t", epson_data->temperature);
#endif
  }

  if (options.gyro_out) {
    if (options.gyro_bit) {
      int gyro_x = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                   (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int gyro_y = (rxByteBuf[idx + 4] << 8 * 3) +
                   (rxByteBuf[idx + 5] << 8 * 2) + (rxByteBuf[idx + 6] << 8) +
                   rxByteBuf[idx + 7];
      int gyro_z = (rxByteBuf[idx + 8] << 8 * 3) +
                   (rxByteBuf[idx + 9] << 8 * 2) + (rxByteBuf[idx + 10] << 8) +
                   rxByteBuf[idx + 11];
      epson_data->gyro_x = (EPSON_GYRO_SF / 65536) * 3.14159 / 180.0 * gyro_x;
      epson_data->gyro_y = (EPSON_GYRO_SF / 65536) * 3.14159 / 180.0 * gyro_y;
      epson_data->gyro_z = (EPSON_GYRO_SF / 65536) * 3.14159 / 180.0 * gyro_z;
      idx += 12;
    } else {
      short gyro_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short gyro_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short gyro_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      epson_data->gyro_x = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_x;
      epson_data->gyro_y = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_y;
      epson_data->gyro_z = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_z;
      idx += 6;
    }
#ifdef DEBUG
    printf("gx: %8.5f\tgy: %8.5f\tgz: %8.5f\t",
           epson_data->gyro_x * 180.0 / 3.14159,
           epson_data->gyro_y * 180.0 / 3.14159,
           epson_data->gyro_z * 180.0 / 3.14159);
#endif
  }

  if (options.accel_out) {
    if (options.accel_bit) {
      int accel_x = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                    (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int accel_y = (rxByteBuf[idx + 4] << 8 * 3) +
                    (rxByteBuf[idx + 5] << 8 * 2) + (rxByteBuf[idx + 6] << 8) +
                    rxByteBuf[idx + 7];
      int accel_z = (rxByteBuf[idx + 8] << 8 * 3) +
                    (rxByteBuf[idx + 9] << 8 * 2) + (rxByteBuf[idx + 10] << 8) +
                    rxByteBuf[idx + 11];
      epson_data->accel_x =
          (EPSON_ACCL_SF / 65536) * 9.80665 / 1000.0 * accel_x;
      epson_data->accel_y =
          (EPSON_ACCL_SF / 65536) * 9.80665 / 1000.0 * accel_y;
      epson_data->accel_z =
          (EPSON_ACCL_SF / 65536) * 9.80665 / 1000.0 * accel_z;
      idx += 12;
    } else {
      short accel_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short accel_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short accel_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      epson_data->accel_x = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_x;
      epson_data->accel_y = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_y;
      epson_data->accel_z = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_z;
      idx += 6;
    }
#ifdef DEBUG
    printf("ax: %8.5f\tay: %8.5f\taz: %8.5f\t",
           epson_data->accel_x * 1000 / 9.80665,
           epson_data->accel_y * 1000 / 9.80665,
           epson_data->accel_z * 1000 / 9.80665);
#endif
  }

  if (options.gyro_delta_out) {
    if (options.gyro_delta_bit) {
      int gyro_delta_x = (rxByteBuf[idx] << 8 * 3) +
                         (rxByteBuf[idx + 1] << 8 * 2) +
                         (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int gyro_delta_y = (rxByteBuf[idx + 4] << 8 * 3) +
                         (rxByteBuf[idx + 5] << 8 * 2) +
                         (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int gyro_delta_z = (rxByteBuf[idx + 8] << 8 * 3) +
                         (rxByteBuf[idx + 9] << 8 * 2) +
                         (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      double da_sf = da_scale_factors[options.dlt_range_ctrl];
      epson_data->gyro_delta_x = gyro_delta_x * (da_sf) / 65536;
      epson_data->gyro_delta_y = gyro_delta_y * (da_sf) / 65536;
      epson_data->gyro_delta_z = gyro_delta_z * (da_sf) / 65536;
      idx += 12;
    } else {
      short gyro_delta_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short gyro_delta_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short gyro_delta_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      double da_sf = da_scale_factors[options.dlt_range_ctrl];
      epson_data->gyro_delta_x = gyro_delta_x * (da_sf);
      epson_data->gyro_delta_y = gyro_delta_y * (da_sf);
      epson_data->gyro_delta_z = gyro_delta_z * (da_sf);
      idx += 6;
    }
#ifdef DEBUG
    printf("dax: %8.5f\tday: %8.5f\tdaz: %8.5f\t", epson_data->gyro_delta_x,
           epson_data->gyro_delta_y, epson_data->gyro_delta_z);
#endif
  }

  if (options.accel_delta_out) {
    if (options.accel_delta_bit) {
      int accel_delta_x = (rxByteBuf[idx] << 8 * 3) +
                          (rxByteBuf[idx + 1] << 8 * 2) +
                          (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int accel_delta_y = (rxByteBuf[idx + 4] << 8 * 3) +
                          (rxByteBuf[idx + 5] << 8 * 2) +
                          (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int accel_delta_z = (rxByteBuf[idx + 8] << 8 * 3) +
                          (rxByteBuf[idx + 9] << 8 * 2) +
                          (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      double dv_sf = dv_scale_factors[options.dlt_range_ctrl];
      epson_data->accel_delta_x = accel_delta_x * (dv_sf) / 65536;
      epson_data->accel_delta_y = accel_delta_y * (dv_sf) / 65536;
      epson_data->accel_delta_z = accel_delta_z * (dv_sf) / 65536;
      idx += 12;
    } else {
      short accel_delta_x = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short accel_delta_y = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short accel_delta_z = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      double dv_sf = dv_scale_factors[options.dlt_range_ctrl];
      epson_data->accel_delta_x = accel_delta_x * (dv_sf);
      epson_data->accel_delta_y = accel_delta_y * (dv_sf);
      epson_data->accel_delta_z = accel_delta_z * (dv_sf);
      idx += 6;
    }
#ifdef DEBUG
    printf("dvx: %8.5f\tdvy: %8.5f\tdvz: %8.5f\t", epson_data->accel_delta_x,
           epson_data->accel_delta_y, epson_data->accel_delta_z);
#endif
  }

  if (options.qtn_out) {
    if (options.qtn_bit) {
      int qtn0 = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int qtn1 = (rxByteBuf[idx + 4] << 8 * 3) + (rxByteBuf[idx + 5] << 8 * 2) +
                 (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      int qtn2 = (rxByteBuf[idx + 8] << 8 * 3) + (rxByteBuf[idx + 9] << 8 * 2) +
                 (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      int qtn3 = (rxByteBuf[idx + 12] << 8 * 3) +
                 (rxByteBuf[idx + 13] << 8 * 2) + (rxByteBuf[idx + 14] << 8) +
                 rxByteBuf[idx + 15];
      epson_data->qtn0 = (double)qtn0 / 1073741824;
      epson_data->qtn1 = (double)qtn1 / 1073741824;
      epson_data->qtn2 = (double)qtn2 / 1073741824;
      epson_data->qtn3 = (double)qtn3 / 1073741824;
      idx += 16;
    } else {
      short qtn0 = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short qtn1 = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short qtn2 = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      short qtn3 = (rxByteBuf[idx + 6] << 8) + rxByteBuf[idx + 7];
      epson_data->qtn0 = (double)qtn0 / 16384;
      epson_data->qtn1 = (double)qtn1 / 16384;
      epson_data->qtn2 = (double)qtn2 / 16384;
      epson_data->qtn3 = (double)qtn3 / 16384;
      idx += 8;
    }
#ifdef DEBUG
    printf("qtn0: %8.5f\tqtn1: %8.5f\tqtn2: %8.5f\tqtn3: %8.5f\t",
           epson_data->qtn0, epson_data->qtn1, epson_data->qtn2,
           epson_data->qtn3);
#endif
  }

  if (options.atti_out) {
    if (options.atti_bit) {
      int roll = (rxByteBuf[idx] << 8 * 3) + (rxByteBuf[idx + 1] << 8 * 2) +
                 (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      int pitch = (rxByteBuf[idx + 4] << 8 * 3) +
                  (rxByteBuf[idx + 5] << 8 * 2) + (rxByteBuf[idx + 6] << 8) +
                  rxByteBuf[idx + 7];
      int yaw = (rxByteBuf[idx + 8] << 8 * 3) + (rxByteBuf[idx + 9] << 8 * 2) +
                (rxByteBuf[idx + 10] << 8) + rxByteBuf[idx + 11];
      epson_data->roll = (EPSON_ATTI_SF / 65536) * 3.14159 / 180.0 * roll;
      epson_data->pitch = (EPSON_ATTI_SF / 65536) * 3.14159 / 180.0 * pitch;
      epson_data->yaw = (EPSON_ATTI_SF / 65536) * 3.14159 / 180.0 * yaw;
      idx += 12;
    } else {
      short roll = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
      short pitch = (rxByteBuf[idx + 2] << 8) + rxByteBuf[idx + 3];
      short yaw = (rxByteBuf[idx + 4] << 8) + rxByteBuf[idx + 5];
      epson_data->roll = EPSON_ATTI_SF * 3.14159 / 180.0 * roll;
      epson_data->pitch = EPSON_ATTI_SF * 3.14159 / 180.0 * pitch;
      epson_data->yaw = EPSON_ATTI_SF * 3.14159 / 180.0 * yaw;
      idx += 6;
    }
#ifdef DEBUG
    printf("roll: %8.3f\tpitch: %8.3f\tyaw: %8.3f\t",
           epson_data->roll * 180.0 / 3.14159,
           epson_data->pitch * 180.0 / 3.14159,
           epson_data->yaw * 180.0 / 3.14159);
#endif
  }

  if (options.gpio_out) {
    unsigned short gpio = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    epson_data->gpio = gpio;
    idx += 2;
#ifdef DEBUG
    printf("gpio: %04x\t", epson_data->gpio);
#endif
  }

#endif
  if (options.count_out) {
    int count = (rxByteBuf[idx] << 8) + rxByteBuf[idx + 1];
    if (options.ext_sel == 1)
      epson_data->count = count * EPSON_COUNT_SF;
    else
      epson_data->count = count;
#ifdef DEBUG
    printf("count: %09d\t", epson_data->count);
#endif
  }
}

/*****************************************************************************
** Function name:       sensorDataReadBurstNOptions
** Description:         Retrieves bytes from the incoming IMU stream of UART
**                      based on expected burst length and searching for START
**                      and END markers. Then calls populateEpsonData() to
**                      post process into struct.
** Parameters:          options - struct describing IMU configuration.
**                      epson_data - struct that is filled with data.
** Return value:        OK or NG
** Notes:
******************************************************************************/
int sensorDataReadBurstNOptions(struct EpsonOptions options,
                                struct EpsonData* epson_data) {
  int byte_length = sensorDataByteLength(options);

#ifdef DEBUG
  printf("Expecting: %d bytes\n", byte_length);
#endif

  int data_length = byte_length - 2;  // exclude the START and END markers
  unsigned char byte;

  while (readComPort(&byte, 1) > 0) {
#ifdef DEBUG
    // printf("state: %d, byte: 0x%02X\n", state, byte);
#endif
    // State machine to seek out START & END markers and then
    // call to populateEpsonData()
    switch (state) {
      case START:
        if (byte == UART_HEADER) state = DATA;
        break;
      case DATA:
        rxByteBuf[data_count] = byte;
        data_count++;
        if (data_count == data_length) state = END;
        break;
      case END:
        data_count = 0;
        state = START;
        if (byte == UART_DELIMITER) {
#ifdef DEBUG
          for (int i = 0; i < data_length; i++) printf("0x%02X ", rxByteBuf[i]);
          printf("\n");
#endif

#ifdef V340PDD0
          // V340 does not support checksum, so just populate sensor data in
          // structure
          populateEpsonData(options, epson_data);
          return OK;
#endif  // V340PDD0
        // All other supported models support checksum
        // If checksum enabled, validate
        // match = populate sensor data structure
        // no match = print error msg and skip current sensor burst data
          if (options.checksum_out == 1) {
            unsigned short calc_checksum = 0;
            for (int i = 0; i < data_length - 2; i += 2) {
              calc_checksum += (rxByteBuf[i] << 8) + rxByteBuf[i + 1];
            }
            unsigned short epson_checksum =
                (rxByteBuf[data_length - 2] << 8) + rxByteBuf[data_length - 1];

            if (calc_checksum != epson_checksum) {
              printf("checksum failed\n");
              return NG;
            }
          }
          populateEpsonData(options, epson_data);
          return OK;
        }
        break;
      default:
        // Should never get here
        printf("Invalid State in Read Burst Processing\n");
    }
  }
  // No byte received in serial port yet
  return NG;
}
