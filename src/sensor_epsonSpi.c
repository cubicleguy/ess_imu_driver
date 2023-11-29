//==============================================================================
//
//  sensor_epsonSpi.c - Epson IMU sensor protocol SPI specific code
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
#include "hcl_gpio.h"
#include "hcl_spi.h"
#include "sensor_epsonCommon.h"

// SPI Interface Timing
// TREADRATE = 40 us min, TCYCLERATE @ 0.5MHz = 32 uS, STALL must be atleast
// (40us - 32us) or 20uS which ever is GREATER (20uS)
#define EPSON_STALL 20                // Microseconds,
#define BURST_STALL1 45               // Microseconds
#define BURST_STALL2 4                // Microseconds
#define selEpson() gpioClr(EPSON_CS)  // For asserting chipselect in SPI IF
#define deselEpson() gpioSet(EPSON_CS)
#define burstStall1() \
  seDelayMicroSecs(   \
      BURST_STALL1)  // For delay after issuing Burst Read Cmd in SPI IF
#define burstStall2() \
  seDelayMicroSecs(BURST_STALL2)  // For delay on consecutive cycles after Burst
                                  // Read Cmd in SPI IF

#ifdef __cplusplus
extern "C" {
#endif

// Function Prototype Specific to SPI IF
void sensorDataReadN(unsigned short[], unsigned int,
                     unsigned char);  // For V340 SPI IF only
void sensorDataReadBurstN(unsigned short[], unsigned int);

#ifdef __cplusplus
}
#endif

static unsigned short rxdata[128];

// Flag and variables to store Delta Velocity & Delta Angle scale factors
static int scale_factors_initialized = 0;
static double dv_scale_factors[16];
static double da_scale_factors[16];

/*****************************************************************************
** Function name:       sensorDataReady
** Description:         For SPI IF check if DataReady is HIGH.
** Parameters:          None
** Return value:        1=DataReady HIGH or 0=DataReady LOW
*****************************************************************************/
int sensorDataReady(void) {
  // Sensor data is ready when the DRDY Pin is HIGH
  return (gpioGetPinLevel(EPSON_DRDY));
}

/*****************************************************************************
** Function name:       registerWriteByteNoID
** Description:         Write Byte to Register = Write Data
**                      to Register (no WIN_ID)
** Parameters:          Register Address, Register Write Byte,
**                      Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByteNoId(unsigned char regAddr, unsigned char regByte,
                           unsigned int verbose) {
  selEpson();
  spiTransfer(regAddr | 0x80);  // msb is 1b for register writes
  spiTransfer(regByte);
  EpsonStall();
  deselEpson();

  if (verbose) {
    printf("\r\nREG[0x%02X] < 0x%02X\t", regAddr, regByte);
  }
}

/*****************************************************************************
** Function name:       registerWriteByte
** Description:         Write Byte to Register = Set WIN_ID,
**                      write Data to Register
**                      NOTE: V340 does not have WINDOW_ID function
**                            winNumber input parameter is ignored
** Parameters:          Window Number, Register Address, Register
**                      WriteDataByte, Verbose Flag
** Return value:        None
*****************************************************************************/
void registerWriteByte(unsigned char winNumber, unsigned char regAddr,
                       unsigned char regByte, unsigned int verbose) {
#if !(defined V340PDD0)
  registerWriteByteNoId(ADDR_WIN_CTRL, winNumber, 0);
#endif

  registerWriteByteNoId(regAddr, regByte, 0);
  if (verbose) {
    printf("\r\nREG[0x%02X(W%01X)] < 0x%02X\t", regAddr, winNumber, regByte);
  }
}

/*****************************************************************************
** Function name:       registerRead16NoId
** Description:         Read 16-bit from Register (No WIN_ID)
** Parameters:          Register Address, Verbose Flag
** Return value:        Register Read Value 16-bit
*****************************************************************************/
unsigned short registerRead16NoId(unsigned char regAddr, unsigned int verbose) {
  short rxData[] = {0x00, 0x00};

  selEpson();
  spiTransfer(regAddr &
              0x7E);  // msb is 0b for register reads & address must be even
  spiTransfer(0x00);
  EpsonStall();

  rxData[0] = spiTransfer(0x00);
  rxData[1] = spiTransfer(0x00);
  EpsonStall();
  deselEpson();

  if (verbose) {
    printf("REG[0x%02X] > 0x%02X%02X\t", regAddr, rxData[0], rxData[1]);
  }
  return (rxData[0] << 8 | rxData[1]);
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
  unsigned short rxData = 0x0000;

#if !(defined V340PDD0)
  registerWriteByteNoId(ADDR_WIN_CTRL, winNumber, 0);
#endif

  rxData = registerRead16NoId(regAddr, 0);
  if (verbose) {
    printf("REG[0x%02X(W%01X)] > 0x%04X\t", regAddr, winNumber, rxData);
  }
  return rxData;
}

/*****************************************************************************
** Function name:       sensorDataReadN
** Description:         Perform SPI sequential register read to acquire sensor
*data
**                      (address is incremented by 2)
**                      NOTE: Use this instead of sensorDataReadBurstN()
**                            for SPI IF V340
** Parameters:          pointer to signed short array, size of array,
**                      register start address
** Return value:        none
** Notes:
** 1. Each register contains 16-bit data, and typical registers to read are:
**
**    For 32-bit sensor output - It is recommended to
*use sensorDataBurstReadN() instead.
**    COUNT [0Ah-0Bh], DUMMY[0Ch-0Dh], TEMPC [0Eh-11h], GX GY GZ [12h-1Ch], AX
*AY AZ [1Eh-28h]
**    Total = Count(2) + Dummy(2) + Temp(4) + GyroXYZ(4*3) + AccelXYZ(4*3) = 32
*bytes = 16 words
**
**    For 16-bit sensor output (V340)
**    NDFLAG [00h-01h], TEMP[02h-03h], GX GY GZ[04h-09h], AX AY AZ[0Ah-0Fh],
*GPIO[10h-11h], COUNT[012h-13h]
**    Total = ND_FLAGS(2) + Temp(2) + GyroXYZ(2*3) + AccelXYZ(2*3) + GPIO(2) +
*Count(2) = 20 bytes = 10 words
**
** 2. Call this function only after detecting DRDY is asserted.
**    This function will send N+1 SPI commands to read N registers.
**    To achieve high performance, we use this function to retrieve burst sensor
*data instead of calling registerRead16().
**    Maximum SPI clock is 2MHz for non-burst SPI reads.
*****************************************************************************/
void sensorDataReadN(unsigned short sensorReadData[], unsigned int readLen,
                     unsigned char regAddr) {
  unsigned int i;

  selEpson();
  spiTransfer(regAddr);
  spiTransfer(0x00);
  EpsonStall();

  for (i = 0; i < readLen; i++) {
    signed int tmp = spiTransfer(regAddr + (2 * (i + 1)));
    sensorReadData[i] = (tmp << 8) + spiTransfer(0x00);
    EpsonStall();
  }
  deselEpson();
}

/*****************************************************************************
** Function name:       sensorDataReadBurstN
** Description:         Perform burst read to acquire sensor data
**                      NOTE: Not supported for SPI IF V340
** Parameters:          pointer to signed short array, size of array
** Return value:        none
** Notes:
** 1. The burst packet consists of 16-bit data units.
**    For 32-bit sensor output, Total = GyroXYZ(4*3) + AccelXYZ(4*3) + Count(2)
*+ Chksum(2) = 28 bytes = 14 words
**    For 16-bit sensor output, Total = ND_FLAGS(2) + Temp(2) + GyroXYZ(2*3) +
*AccelXYZ(2*3) + GPIO(2) + Count(2) = 20 bytes = 10 words
** 2. For SPI interface, call this function only after detecting DRDY is
*asserted.
**    For SPI interface, this function will send N+1 SPI commands to read N
*registers.
**    Maximum SPI clock is 1MHz for burst SPI reads.
** 3. No checksum verification is performed (in this function)
**    To achieve high performance, we use this function to retrieve burst sensor
*data instead of calling registerRead16().
**
*****************************************************************************/
void sensorDataReadBurstN(unsigned short sensorReadData[],
                          unsigned int readLen) {
  unsigned int i;

  selEpson();
  spiTransfer(CMD_BURST);
  spiTransfer(0x00);
  burstStall1();

  for (i = 0; i < readLen; i++) {
    signed short tmp = spiTransfer(0x00);
    sensorReadData[i] = (tmp << 8) + spiTransfer(0x00);
#ifdef DEBUG
    printf("\r\n0x%04x:[0x%04X]", i, sensorReadData[i]);
#endif
    burstStall2();
  }
  deselEpson();
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
  unsigned short ndflags = rxdata[idx];
  epson_data->ndflags = ndflags;
  idx++;

  short temp = rxdata[idx];
  epson_data->temperature = (temp - 2634) * EPSON_TEMP_SF + 25;
  idx++;

  short gyro_x = rxdata[idx];
  short gyro_y = rxdata[idx + 1];
  short gyro_z = rxdata[idx + 2];
  epson_data->gyro_x = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_x;
  epson_data->gyro_y = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_y;
  epson_data->gyro_z = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_z;
  idx += 3;

  short accel_x = rxdata[idx];
  short accel_y = rxdata[idx + 1];
  short accel_z = rxdata[idx + 2];
  epson_data->accel_x = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_x;
  epson_data->accel_y = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_y;
  epson_data->accel_z = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_z;
  idx += 3;
  unsigned short gpio = rxdata[idx];
  idx++;
  epson_data->gpio = gpio;

#else
  // parsing of data fields applying conversion factor if applicable
  if (options.flag_out) {
    unsigned short ndflags = rxdata[idx];
    epson_data->ndflags = ndflags;
    idx++;
#ifdef DEBUG
    printf("ndflag: %04x\t", epson_data->ndflags);
#endif
  }

  if (options.temp_out) {
    if (options.temp_bit) {
      int temp = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
#if defined G330PDG0 || defined G366PDG0 || defined G370PDG0 || defined G370PDT0
      epson_data->temperature = (temp)*EPSON_TEMP_SF / 65536 + 25;
#else
      epson_data->temperature = (temp - 172621824) * EPSON_TEMP_SF / 65536 + 25;
#endif  // defined G330PDG0 || defined G366PDG0 || defined G370PDG0 || defined
        // G370PDT0
      idx += 2;
    } else {
      short temp = rxdata[idx];
#if defined G330PDG0 || defined G366PDG0 || defined G370PDG0 || defined G370PDT0
      epson_data->temperature = (temp)*EPSON_TEMP_SF + 25;
#else
      epson_data->temperature = (temp - 2634) * EPSON_TEMP_SF + 25;
#endif  // defined G330PDG0 || defined G366PDG0 || defined G370PDG0 || defined
        // G370PDT0
      idx++;
    }
#ifdef DEBUG
    printf("tempC: %8.3f\t", epson_data->temperature);
#endif
  }

  if (options.gyro_out) {
    if (options.gyro_bit) {
      int gyro_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int gyro_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int gyro_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      epson_data->gyro_x = (EPSON_GYRO_SF / 65536) * 3.14159 / 180.0 * gyro_x;
      epson_data->gyro_y = (EPSON_GYRO_SF / 65536) * 3.14159 / 180.0 * gyro_y;
      epson_data->gyro_z = (EPSON_GYRO_SF / 65536) * 3.14159 / 180.0 * gyro_z;
      idx += 6;
    } else {
      short gyro_x = rxdata[idx];
      short gyro_y = rxdata[idx + 1];
      short gyro_z = rxdata[idx + 2];
      epson_data->gyro_x = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_x;
      epson_data->gyro_y = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_y;
      epson_data->gyro_z = EPSON_GYRO_SF * 3.14159 / 180.0 * gyro_z;
      idx += 3;
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
      int accel_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int accel_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int accel_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      epson_data->accel_x =
          (EPSON_ACCL_SF / 65536) * 9.80665 / 1000.0 * accel_x;
      epson_data->accel_y =
          (EPSON_ACCL_SF / 65536) * 9.80665 / 1000.0 * accel_y;
      epson_data->accel_z =
          (EPSON_ACCL_SF / 65536) * 9.80665 / 1000.0 * accel_z;
      idx += 6;
    } else {
      short accel_x = rxdata[idx];
      short accel_y = rxdata[idx + 1];
      short accel_z = rxdata[idx + 2];
      epson_data->accel_x = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_x;
      epson_data->accel_y = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_y;
      epson_data->accel_z = (EPSON_ACCL_SF)*9.80665 / 1000.0 * accel_z;
      idx += 3;
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
      int gyro_delta_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int gyro_delta_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int gyro_delta_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      double da_sf = da_scale_factors[options.dlt_range_ctrl];
      epson_data->gyro_delta_x = gyro_delta_x * (da_sf) / 65536;
      epson_data->gyro_delta_y = gyro_delta_y * (da_sf) / 65536;
      epson_data->gyro_delta_z = gyro_delta_z * (da_sf) / 65536;
      idx += 6;
    } else {
      short gyro_delta_x = rxdata[idx];
      short gyro_delta_y = rxdata[idx + 1];
      short gyro_delta_z = rxdata[idx + 2];
      double da_sf = da_scale_factors[options.dlt_range_ctrl];
      epson_data->gyro_delta_x = gyro_delta_x * (da_sf);
      epson_data->gyro_delta_y = gyro_delta_y * (da_sf);
      epson_data->gyro_delta_z = gyro_delta_z * (da_sf);
      idx += 3;
    }
#ifdef DEBUG
    printf("dax: %8.5f\tday: %8.5f\tdaz: %8.5f\t", epson_data->gyro_delta_x,
           epson_data->gyro_delta_y, epson_data->gyro_delta_z);
#endif
  }

  if (options.accel_delta_out) {
    if (options.accel_delta_bit) {
      int accel_delta_x = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int accel_delta_y = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int accel_delta_z = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      double dv_sf = dv_scale_factors[options.dlt_range_ctrl];
      epson_data->accel_delta_x = accel_delta_x * (dv_sf) / 65536;
      epson_data->accel_delta_y = accel_delta_y * (dv_sf) / 65536;
      epson_data->accel_delta_z = accel_delta_z * (dv_sf) / 65536;
      idx += 6;
    } else {
      short accel_delta_x = rxdata[idx];
      short accel_delta_y = rxdata[idx + 1];
      short accel_delta_z = rxdata[idx + 2];
      double dv_sf = dv_scale_factors[options.dlt_range_ctrl];
      epson_data->accel_delta_x = accel_delta_x * (dv_sf);
      epson_data->accel_delta_y = accel_delta_y * (dv_sf);
      epson_data->accel_delta_z = accel_delta_z * (dv_sf);
      idx += 3;
    }
#ifdef DEBUG
    printf("dvx: %8.5f\tdvy: %8.5f\tdvz: %8.5f\t", epson_data->accel_delta_x,
           epson_data->accel_delta_y, epson_data->accel_delta_z);
#endif
  }

  if (options.qtn_out) {
    if (options.qtn_bit) {
      int qtn0 = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int qtn1 = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int qtn2 = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      int qtn3 = (rxdata[idx + 6] << 8 * 2) + rxdata[idx + 7];
      epson_data->qtn0 = (double)qtn0 / 1073741824;
      epson_data->qtn1 = (double)qtn1 / 1073741824;
      epson_data->qtn2 = (double)qtn2 / 1073741824;
      epson_data->qtn3 = (double)qtn3 / 1073741824;
      idx += 8;
    } else {
      short qtn0 = rxdata[idx];
      short qtn1 = rxdata[idx + 1];
      short qtn2 = rxdata[idx + 2];
      short qtn3 = rxdata[idx + 3];
      epson_data->qtn0 = (double)qtn0 / 16384;
      epson_data->qtn1 = (double)qtn1 / 16384;
      epson_data->qtn2 = (double)qtn2 / 16384;
      epson_data->qtn3 = (double)qtn3 / 16384;
      idx += 4;
    }
#ifdef DEBUG
    printf("qtn0: %8.5f\tqtn1: %8.5f\tqtn2: %8.5f\tqtn3: %8.5f\t",
           epson_data->qtn0, epson_data->qtn1, epson_data->qtn2,
           epson_data->qtn3);
#endif
  }

  if (options.atti_out) {
    if (options.atti_bit) {
      int roll = (rxdata[idx] << 8 * 2) + rxdata[idx + 1];
      int pitch = (rxdata[idx + 2] << 8 * 2) + rxdata[idx + 3];
      int yaw = (rxdata[idx + 4] << 8 * 2) + rxdata[idx + 5];
      epson_data->roll = (EPSON_ATTI_SF / 65536) * 3.14159 / 180.0 * roll;
      epson_data->pitch = (EPSON_ATTI_SF / 65536) * 3.14159 / 180.0 * pitch;
      epson_data->yaw = (EPSON_ATTI_SF / 65536) * 3.14159 / 180.0 * yaw;
      idx += 6;
    } else {
      short roll = rxdata[idx];
      short pitch = rxdata[idx + 1];
      short yaw = rxdata[idx + 2];
      epson_data->roll = EPSON_ATTI_SF * 3.14159 / 180.0 * roll;
      epson_data->pitch = EPSON_ATTI_SF * 3.14159 / 180.0 * pitch;
      epson_data->yaw = EPSON_ATTI_SF * 3.14159 / 180.0 * yaw;
      idx += 3;
    }
#ifdef DEBUG
    printf("roll: %8.3f\tpitch: %8.3f\tyaw: %8.3f\t",
           epson_data->roll * 180.0 / 3.14159,
           epson_data->pitch * 180.0 / 3.14159,
           epson_data->yaw * 180.0 / 3.14159);
#endif
  }

  if (options.gpio_out) {
    unsigned short gpio = rxdata[idx];
    epson_data->gpio = gpio;
    idx++;
#ifdef DEBUG
    printf("gpio: %04x\t", epson_data->gpio);
#endif
  }

#endif
  if (options.count_out) {
    int count = rxdata[idx];
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
** Description:         If DataReady is active retrieves 1 packet from the
**                      incoming IMU stream based on expected burst length
**                      for SPI IF. Then calls populateEpsonData() to
**                      post process into struct.
** Parameters:          options - struct describing IMU configuration.
**                      epson_data - struct that is filled with data.
** Return value:        OK or NG (DataReady retry exceed or checksum error)
** Notes:
******************************************************************************/
int sensorDataReadBurstNOptions(struct EpsonOptions options,
                                struct EpsonData* epson_data) {
  // Wait for DataReady or return NG if retry timeout
  int retryCount = 50000;
  do {
    seDelayMicroSecs(10);
    retryCount--;
    if (retryCount == 0) {
      return NG;
    }
  } while (!sensorDataReady());

  unsigned int data_length = sensorDataByteLength(options) / 2;

  // Burst read the sensor data based on calculated burst size from options
  // struct

#ifdef V340PDD0
  // Burst read routine for V340
  sensorDataReadN(rxdata, data_length, 0x00);
  // V340 does not support checksum, so just populate sensor data in structure
  populateEpsonData(options, epson_data);
  return OK;
#endif  // V340PDD0

  // Burst read routine for all other IMU models
  sensorDataReadBurstN(rxdata, data_length);
  // If checksum enabled, validate checksum and populate sensor data in
  // structure
  if (options.checksum_out == 1) {
    unsigned short calc_checksum = 0;
    for (unsigned int i = 0; i < data_length - 1; i++) {
      calc_checksum += rxdata[i];
    }
    unsigned short epson_checksum = rxdata[data_length - 1];

    if (calc_checksum != epson_checksum) {
      printf("checksum failed\n");
      return NG;
    }
  }
  populateEpsonData(options, epson_data);
  return OK;
}
