//==============================================================================
//
//  sensor_epsonCommon.c - Epson IMU sensor protocol specific code common
//                      for all IMU models
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

#include "hcl.h"
#include "hcl_gpio.h"
#include "sensor_epsonCommon.h"

/*****************************************************************************
** Function name:       sensorHWReset
** Description:         Toggle the RESET pin, delay, wait for NOT_READY bit=0
**                      This is only applicable on embedded platforms with
**                      GPIO pin connected to IMU RESET#
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorHWReset(void) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  gpioSet(EPSON_RESET);  // RESET pin HIGH
  seDelayMS(DELAY_EPSON_RESET);
  gpioClr(EPSON_RESET);  // Assert RESET (LOW)
  seDelayMS(DELAY_EPSON_RESET);
  gpioSet(EPSON_RESET);  // Deassert RESET (HIGH)
  seDelayMS(EPSON_POWER_ON_DELAY);

  // Poll NOT_READY bit every 1msec until returns 0
  // Exit after specified retries
  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, debug);
    seDelayMicroSecs(1000);
    retryCount--;
  } while ((rxData & 0x0400) == 0x0400 && (retryCount != 0));

  if (retryCount == 0) {
    printf("\r\n...Error: NOT_READY stuck HIGH.");
    return NG;
  }
  return OK;
}

/*****************************************************************************
** Function name:       sensorPowerOn
** Description:         Initial startup, Goto Config Mode (sanity),
**                      Check for Hardware Error Flags
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorPowerOn(void) {
  unsigned short rxData = 0xFFFF;
  unsigned int debug = FALSE;
  unsigned short retryCount = 3000;

  // Safety Measure, Force Exit of Sampling Mode
  do {
    registerWriteByte(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_END_SAMPLING, debug);
    rxData = registerRead16(CMD_WINDOW0, ADDR_MODE_CTRL_LO, debug);
    seDelayMicroSecs(1000);
    retryCount--;
  } while ((rxData & 0x0400) == 0x0000 && (retryCount != 0));

  if (retryCount == 0) {
    printf("\r\n...Error: Stuck in Sampling Mode.");
  }

  // Hardware Reset if connected, and check for NOT_READY flag
  if (!sensorHWReset()) {
    return NG;
  }

  // Check for error flags
  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  if (rxData == 0x0000)
    return OK;
  else
    return NG;
}

/*****************************************************************************
** Function name:       sensorStart
** Description:         Start sensor sampling (goto Sampling Mode)
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorStart(void) {
  unsigned int debug = FALSE;

  registerWriteByte(CMD_WINDOW0, ADDR_MODE_CTRL_HI, CMD_BEGIN_SAMPLING, debug);
  printf("\r\n...Sensor start.");
}

/*****************************************************************************
** Function name:       sensorStop
** Description:         Stop sensor sampling (goto Config Mode)
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorStop(void) {
  unsigned int debug = FALSE;

  registerWriteByteNoId(ADDR_MODE_CTRL_HI, CMD_END_SAMPLING, debug);
  seDelayMicroSecs(
      200000);  // Provide 200msec for sensor to finish sending sample
  printf("\r\n...Sensor stop.");
}

/*****************************************************************************
** Function name:       sensorReset
** Description:         Send Software Reset to Sensor + Delay 800 msec
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorReset(void) {
  unsigned int debug = FALSE;

  printf("\r\n...Software Reset begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_SOFTRESET, debug);
  seDelayMS(EPSON_POWER_ON_DELAY);
  printf("\r\n...Software Reset complete.");
}

/*****************************************************************************
** Function name:       sensorFlashTest
** Description:         Send Flashtest command to Sensor and check status
**                      NOTE: Not supported for V340PDD0
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorFlashTest(void) {
#if defined V340PDD0
  // always return OK
#else
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  printf("\r\n...Flash test begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_FLASHTEST, debug);
  seDelayMS(EPSON_FLASH_TEST_DELAY);
  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & 0x0800) == 0x0800 && (retryCount != 0));
  if (retryCount == 0) {
    printf("\r\n...Error: Flashtest bit did not return to 0b.");
    return NG;
  }

  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Flash test complete.");

  if ((rxData & 0x0004) != 0x0000) return NG;

#endif
  return OK;
}

/*****************************************************************************
** Function name:       sensorSelfTest
** Description:         Send SelfTest command to Sensor and check status
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorSelfTest(void) {
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  printf("\r\n...Self test begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_HI, CMD_SELFTEST, debug);
  seDelayMS(EPSON_SELF_TEST_DELAY);
  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_MSC_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & 0x0400) == 0x0400 && (retryCount != 0));
  if (retryCount == 0) {
    printf("\r\n...Error: Self test bit did not return to 0b.");
    return NG;
  }

  rxData = registerRead16(CMD_WINDOW0, ADDR_DIAG_STAT, debug);
  printf("\r\n...Self test complete.");

  if ((rxData & 0x0002) == 0x0000)
    return OK;
  else
    return NG;
}

/*****************************************************************************
** Function name:       sensorInitialBackup
** Description:         Send InitialBackup command (restore defaults) to Sensor
**                      and check status
**                      NOTE: Only supported for G365/G370
** Parameters:          None
** Return value:        OK or NG
*****************************************************************************/
int sensorInitialBackup(void) {
#if defined G354PDH0 || defined G364PDCA || defined G364PDC0 || \
    defined V340PDD0 || defined G320PDG0
  // always return OK
#else
  unsigned int debug = FALSE;
  unsigned short rxData;
  unsigned short retryCount = 3000;

  printf("\r\n...InitialBackup begin.");
  registerWriteByte(CMD_WINDOW1, ADDR_GLOB_CMD_LO, CMD_INITIAL_BACKUP, debug);
  seDelayMS(EPSON_FLASH_TEST_DELAY);

  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_GLOB_CMD_LO, debug);
    retryCount--;
  } while ((rxData & 0x0010) == 0x0010 && (retryCount != 0));
  if (retryCount == 0) {
    printf("\r\n...Error: InitialBackup bit did not return to 0b.");
    return NG;
  }

  printf("\r\n...Initial Backup complete.");
#endif
  return OK;
}

/*****************************************************************************
** Function name:       sensorDataByteLength
** Description:         Determines the sensor burst read packet data length
**                      based on the parameters in the EpsonOptions struct.
** Parameters:          options - struct describing IMU configuration.
** Return value:        data byte length
*****************************************************************************/
unsigned int sensorDataByteLength(struct EpsonOptions options) {
  unsigned int length = 0;

#ifdef V340PDD0
  // V340 has fixed packet format with optional 16-bit count value
  length = 18;

#else
  // 16 bit ND_EA FLAG
  if (options.flag_out) length += 2;

  // 16 or 32 bit Temperature Output
  if (options.temp_out) {
    if (options.temp_bit)
      length += 4;
    else
      length += 2;
  }

  // 16 or 32 bit Gyro X, Y, Z Output
  if (options.gyro_out) {
    if (options.gyro_bit)
      length += 12;
    else
      length += 6;
  }

  // 16 or 32 bit Accl X, Y, Z Output
  if (options.accel_out) {
    if (options.accel_bit)
      length += 12;
    else
      length += 6;
  }

  // 16 or 32 bit Delta Angle X, Y, Z Output
  if (options.gyro_delta_out) {
    if (options.gyro_delta_bit)
      length += 12;
    else
      length += 6;
  }

  // 16 or 32 bit Delta Velocity X, Y, Z Output
  if (options.accel_delta_out) {
    if (options.accel_delta_bit)
      length += 12;
    else
      length += 6;
  }

#if defined G325PDF1 || defined G365PDC1 || defined G365PDF1 || \
    defined G330PDG0 || defined G366PDG0
  // 16 or 32 bit Quaternion1,2,3,4 Output
  // NOTE: Only supported by G330/G366/G365PDCx/G365PDFx/G325PDFx
  if (options.qtn_out) {
    if (options.qtn_bit)
      length += 16;
    else
      length += 8;
  }
  // 16 or 32 bit Attitude X, Y, Z Output
  // NOTE: Only supported by G330/G366/G365PDCx/G365PDFx/G325PDFx
  if (options.atti_out) {
    if (options.atti_bit)
      length += 12;
    else
      length += 6;
  }
#endif

  // 16 bit GPIO status output
  if (options.gpio_out) {
    length += 2;
  }
  if (options.checksum_out) {
    length += 2;
  }
#endif

  // 16 bit Count output
  if (options.count_out) {
    length += 2;
  }

#if !defined SPI
  // For Start and End byte when using UART interface
  length += 2;
#endif

  return length;
}

/*****************************************************************************
** Function name:       sensorDummyWrite
** Description:         Sets the WINDOW_ID of IMU to 0
**                      This is a workaround to flush the UART port on embedded
**                      Linux platform to prevent hanging if the first register
**                      access is register read
** Parameters:          None
** Return value:        None
*****************************************************************************/
void sensorDummyWrite(void) {
  unsigned int debug = FALSE;

  seDelayMicroSecs(100000);
  registerWriteByteNoId(ADDR_WIN_CTRL, 0x00, debug);
  seDelayMicroSecs(100000);
  printf("\r\n...sensorDummyWrite.");
}

/*****************************************************************************
** Function name:       getProductId
** Description:         Updates char array with Product ID ASCII
**
** Parameters:          pointer to String
** Return value:        pointer to String
*****************************************************************************/
char* getProductId(char* pcharArr) {
  unsigned short prod_id1 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID1, FALSE);
  unsigned short prod_id2 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID2, FALSE);
  unsigned short prod_id3 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID3, FALSE);
  unsigned short prod_id4 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID4, FALSE);

  pcharArr[0] = (char)prod_id1;
  pcharArr[1] = (char)(prod_id1 >> 8);
  pcharArr[2] = (char)prod_id2;
  pcharArr[3] = (char)(prod_id2 >> 8);
  pcharArr[4] = (char)prod_id3;
  pcharArr[5] = (char)(prod_id3 >> 8);
  pcharArr[6] = (char)prod_id4;
  pcharArr[7] = (char)(prod_id4 >> 8);
  pcharArr[8] = '\0';

  return (pcharArr);
}

/*****************************************************************************
** Function name:       getSerialId
** Description:         Updates char array with Serial ID ASCII
**
** Parameters:          pointer to String
** Return value:        pointer to String
*****************************************************************************/
char* getSerialId(char* pcharArr) {
  unsigned short ser_num1 =
      registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1, FALSE);
  unsigned short ser_num2 =
      registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM2, FALSE);
  unsigned short ser_num3 =
      registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM3, FALSE);
  unsigned short ser_num4 =
      registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM4, FALSE);

  pcharArr[0] = (char)ser_num1;
  pcharArr[1] = (char)(ser_num1 >> 8);
  pcharArr[2] = (char)ser_num2;
  pcharArr[3] = (char)(ser_num2 >> 8);
  pcharArr[4] = (char)ser_num3;
  pcharArr[5] = (char)(ser_num3 >> 8);
  pcharArr[6] = (char)ser_num4;
  pcharArr[7] = (char)(ser_num4 >> 8);
  pcharArr[8] = '\0';

  return pcharArr;
}
