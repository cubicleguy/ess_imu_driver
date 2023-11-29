//==============================================================================
//
//  sensor_epsonV340.c - Epson IMU sensor protocol specific code for V340
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

  // COUNT_CTRL
  // V340 only has option to enable or disable counter with burst read packets
  // Otherwise the packet format and data fields are fixed

  int count_ctrl_lo = (options.count_out & 0x1);

  // POL_CTRL
  // If these bits are set, then the axis values are reverse polarity

  int pol_ctrl_lo =
      (options.invert_zaccel & 0x01) << 1 |
      (options.invert_yaccel & 0x01) << 2 |
      (options.invert_xaccel & 0x01) << 3 | (options.invert_zgyro & 0x01) << 4 |
      (options.invert_ygyro & 0x01) << 5 | (options.invert_xgyro & 0x01) << 6;

  registerWriteByte(CMD_WINDOW1, ADDR_SIG_CTRL_HI, sig_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_MSC_CTRL_LO, msc_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_SMPL_CTRL_HI, smpl_ctrl_hi, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_POL_CTRL_LO, pol_ctrl_lo, debug);
  registerWriteByte(CMD_WINDOW1, ADDR_FILTER_CTRL_LO, filter_ctrl_lo, debug);

#ifdef SPI  // Always disable UART_AUTO mode for burst reading when using SPI IF
  registerWriteByte(CMD_WINDOW1, ADDR_UART_CTRL_LO, 0x00, debug);
#else
  registerWriteByte(CMD_WINDOW1, ADDR_UART_CTRL_LO, 0x01, debug);
#endif

  registerWriteByte(CMD_WINDOW1, ADDR_COUNT_CTRL_LO, count_ctrl_lo, debug);

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
  registerRead16(0x00, 0x00, debug);
  registerRead16(0x00, 0x02, debug);
  registerRead16(0x00, 0x04, debug);
  printf("\r\n");
  registerRead16(0x00, 0x06, debug);
  registerRead16(0x00, 0x08, debug);
  registerRead16(0x00, 0x0A, debug);
  printf("\r\n");
  registerRead16(0x00, 0x0C, debug);
  registerRead16(0x00, 0x0E, debug);
  registerRead16(0x00, 0x10, debug);
  printf("\r\n");
  registerRead16(0x00, 0x12, debug);
  registerRead16(0x00, 0x32, debug);
  registerRead16(0x00, 0x34, debug);
  printf("\r\n");
  registerRead16(0x00, 0x36, debug);
  registerRead16(0x00, 0x38, debug);
  registerRead16(0x00, 0x3A, debug);
  printf("\r\n");
  registerRead16(0x00, 0x3C, debug);
  registerRead16(0x00, 0x3E, debug);
  registerRead16(0x00, 0x50, debug);
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
}
