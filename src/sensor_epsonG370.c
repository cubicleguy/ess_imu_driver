//==============================================================================
//
//  sensor_epsonG370.c - Epson IMU sensor protocol specific code for G370
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
#include <stdint.h>

#include "hcl.h"
#include "sensor_epsonCommon.h"

/*****************************************************************************
** Function name:       sensorInitOptions
** Description:         Initialize the sensor hardware to desired settings
**                      based on EpsonOptions
** Parameters:          struct EpsonOptions
** Return value:        OK or NG
**
*****************************************************************************/
int sensorInitOptions(struct EpsonOptions options) {
  unsigned int debug = FALSE;

  // SIG_CTRL
  // ND flags for gyro_delta_out X,Y,Z are enabled if gyro_delta_out is enabled
  // ND flags for accel_delta_out X,Y,Z are enabled if accel_delta_out is
  // enabled

  int sig_ctrl_lo = (options.gyro_delta_out & 0x01) << 2 |
                    (options.gyro_delta_out & 0x01) << 3 |
                    (options.gyro_delta_out & 0x01) << 4 |
                    (options.accel_delta_out & 0x01) << 5 |
                    (options.accel_delta_out & 0x01) << 6 |
                    (options.accel_delta_out & 0x01) << 7;

  // ND flags for gyro_out X,Y,Z are enabled if gyro_out is enabled
  // ND flags for accel_out X,Y,Z are enabled if accel_out is enabled
  // ND flag for temp_out is enabled if temp_out is enabled

  int sig_ctrl_hi =
      (options.accel_out & 0x01) << 1 | (options.accel_out & 0x01) << 2 |
      (options.accel_out & 0x01) << 3 | (options.gyro_out & 0x01) << 4 |
      (options.gyro_out & 0x01) << 5 | (options.gyro_out & 0x01) << 6 |
      (options.temp_out & 0x01) << 7;

  // MSC_CTRL
  // Configure DRDY function (if needed) & EXT pin function on GPIO2 (if needed)
  // External Counter Reset is typically used when GPIO2 is connected to a
  // PPS-like signal

  int msc_ctrl_lo =
      (options.drdy_pol & 0x01) << 1 | (options.drdy_on & 0x01) << 2 |
      (options.ext_pol & 0x01) << 5 | (options.ext_sel & 0x03) << 6;

  // SMPL_CTRL
  // Configures the Data Output Rate of the IMU.
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations

  int smpl_ctrl_hi = (options.dout_rate & 0x0F);

  // FILTER_CTRL
  // Configures the FIR filter of the IMU.
  // Refer to Datasheet for valid Data Output Rate & Filter Setting combinations
  // If External Trigger is enabled on GPIO2, then it is recommended to set the
  // the FILTER_SEL=0. And program the GYRO_LPF_FC & ACCL_LPF_FC to meet Nyquist
  // based on the Trigger Frequency.

  int filter_ctrl_lo = (options.filter_sel & 0x1F);

  // BURST_CTRL1
  // These enable or disable certain data fields in the burst read packet

  int burst_ctrl1_lo = (options.checksum_out & 0x1) |
                       (options.count_out & 0x1) << 1 |
                       (options.gpio_out & 0x01) << 2;

  int burst_ctrl1_hi =
      (options.gyro_delta_out & 0x1) << 2 |
      (options.accel_delta_out & 0x01) << 3 | (options.accel_out & 0x01) << 4 |
      (options.gyro_out & 0x01) << 5 | (options.temp_out & 0x01) << 6 |
      (options.flag_out & 0x01) << 7;

  // BURST_CTRL2
  // If certain data fields are enabled, these bits determine if the
  // data fields are 16 or 32 bit

  int burst_ctrl2_hi =
      (options.gyro_delta_bit & 0x01) << 2 |
      (options.accel_delta_bit & 0x01) << 3 | (options.accel_bit & 0x01) << 4 |
      (options.gyro_bit & 0x01) << 5 | (options.temp_bit & 0x01) << 6;

  // DLT_CTRL
  // Enable or disable Delta Angle/Velocity overflow flag in DIAG_STAT
  // Set A_RANGE_CTRL to 1, if macro defined
  // Set the Delta Angle/Velocity Scale Factor

#ifdef ACCL_RANGE_16G
  int dlt_ctrl_hi = (options.dlt_ovf_en & 0x01) << 1 | 1;
#else
  int dlt_ctrl_hi = (options.dlt_ovf_en & 0x01) << 1;
#endif  // ACCL_RANGE_16G
  int dlt_ctrl_lo =
      (options.dlt_range_ctrl & 0x0F) << 4 | (options.dlt_range_ctrl & 0x0F);

  // ATTI_CTRL
  // For G370, Attitude Output is not supported.
  // This is only for Delta Angle/Velocity function if enabled.

  int atti_ctrl_hi = (options.gyro_delta_out & 0x01) << 1;

  // POL_CTRL
  // If these bits are set, then the axis values are reverse polarity

  int pol_ctrl_lo =
      (options.invert_zaccel & 0x01) << 1 |
      (options.invert_yaccel & 0x01) << 2 |
      (options.invert_xaccel & 0x01) << 3 | (options.invert_zgyro & 0x01) << 4 |
      (options.invert_ygyro & 0x01) << 5 | (options.invert_xgyro & 0x01) << 6;

  registerWriteByte(CMD_WINDOW1, ADDR_SIG_CTRL_HI, sig_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_SIG_CTRL_LO, sig_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_LO, msc_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_POL_CTRL_LO, pol_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, filter_ctrl_lo, debug);

  // All models except V340
  // Delay for filter config
  seDelayMS(EPSON_FILTER_DELAY);

  // Check that the FILTER_BUSY bit returns 0
  unsigned short rxData;
  unsigned short retryCount = 3000;
  do {
    rxData = registerRead16(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, debug);
    retryCount--;
  } while ((rxData & 0x0020) == 0x0020 && (retryCount != 0));

  if (retryCount == 0) {
    printf("\r\n...Error: Filter busy bit did not return to 0b.");
    return NG;
  }

#ifdef SPI  // Always disable UART_AUTO mode for burst reading when using SPI IF
  registerWriteByte(CMD_WINDOW1, ADDR_UART_CTRL_LO, 0x00, debug);
#else
  registerWriteByte(CMD_WINDOW1, ADDR_UART_CTRL_LO, 0x01, debug);
#endif

  registerWriteByte(CMD_WINDOW1, ADDR_BURST_CTRL1_LO, burst_ctrl1_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_BURST_CTRL1_HI, burst_ctrl1_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_BURST_CTRL2_HI, burst_ctrl2_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_DLT_CTRL_HI, dlt_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_DLT_CTRL_LO, dlt_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_ATTI_CTRL_HI, atti_ctrl_hi, debug);

  return OK;
}

/*****************************************************************************
** Function name:       registerDump
** Description:         Read all registers for debug purpose
** Parameters:          None
** Return value:        None
*****************************************************************************/
void registerDump(void) {
  unsigned int debug = TRUE;
  printf("\r\nRegister Dump:\r\n");
  registerRead16(0x00, 0x02, debug);
  registerRead16(0x00, 0x04, debug);
  registerRead16(0x00, 0x06, debug);
  printf("\r\n");
  registerRead16(0x00, 0x08, debug);
  registerRead16(0x00, 0x0A, debug);
  registerRead16(0x00, 0x0C, debug);
  printf("\r\n");
  registerRead16(0x00, 0x0E, debug);
  registerRead16(0x00, 0x10, debug);
  registerRead16(0x00, 0x12, debug);
  printf("\r\n");
  registerRead16(0x00, 0x14, debug);
  registerRead16(0x00, 0x16, debug);
  registerRead16(0x00, 0x18, debug);
  printf("\r\n");
  registerRead16(0x00, 0x1A, debug);
  registerRead16(0x00, 0x1C, debug);
  registerRead16(0x00, 0x1E, debug);
  printf("\r\n");
  registerRead16(0x00, 0x20, debug);
  registerRead16(0x00, 0x22, debug);
  registerRead16(0x00, 0x24, debug);
  printf("\r\n");
  registerRead16(0x00, 0x26, debug);
  registerRead16(0x00, 0x28, debug);

#if defined G370PDF1 || defined G370PDS0
  registerRead16(0x00, 0x2B, debug);
  registerRead16(0x00, 0x4C, debug);
#endif  // defined G370PDF1 || defined G370PDS0

  printf("\r\n");
  registerRead16(0x00, 0x64, debug);
  registerRead16(0x00, 0x66, debug);
  registerRead16(0x00, 0x68, debug);
  printf("\r\n");
  registerRead16(0x00, 0x6A, debug);
  registerRead16(0x00, 0x6C, debug);
  registerRead16(0x00, 0x6E, debug);
  printf("\r\n");
  registerRead16(0x00, 0x70, debug);
  registerRead16(0x00, 0x72, debug);
  registerRead16(0x00, 0x74, debug);
  printf("\r\n");
  registerRead16(0x00, 0x76, debug);
  registerRead16(0x00, 0x78, debug);
  registerRead16(0x00, 0x7A, debug);
  printf("\r\n");
  registerRead16(0x01, 0x00, debug);
  registerRead16(0x01, 0x02, debug);
  registerRead16(0x01, 0x04, debug);
  printf("\r\n");
  registerRead16(0x01, 0x06, debug);
  registerRead16(0x01, 0x08, debug);
  registerRead16(0x01, 0x0A, debug);
  printf("\r\n");
  registerRead16(0x01, 0x0C, debug);
  registerRead16(0x01, 0x0E, debug);
  registerRead16(0x01, 0x10, debug);
  printf("\r\n");
  registerRead16(0x01, 0x12, debug);
  registerRead16(0x01, 0x14, debug);
  registerRead16(0x01, 0x16, debug);
  printf("\r\n");

#ifdef G370PDF0
  registerRead16(0x01, 0x18, debug);
  printf("\r\n");
#endif  // G370PDF0

  registerRead16(0x01, 0x6A, debug);
  registerRead16(0x01, 0x6C, debug);
  registerRead16(0x01, 0x6E, debug);
  printf("\r\n");
  registerRead16(0x01, 0x70, debug);
  registerRead16(0x01, 0x72, debug);
  registerRead16(0x01, 0x74, debug);
  printf("\r\n");
  registerRead16(0x01, 0x76, debug);
  registerRead16(0x01, 0x78, debug);
  registerRead16(0x01, 0x7A, debug);
  printf("\r\n");
  registerRead16(0x01, 0x7E, debug);
  printf("\r\n");
}
