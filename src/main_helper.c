//==============================================================================
//
//  main_helper.c - Epson IMU helper functions for console utilities
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

#include "main_helper.h"
#include "sensor_epsonCommon.h"

/*****************************************************************************
** Function name:       printHeaderRow
** Description:         Output header row
**                      based on EpsonOptions
** Parameters:          fp File pointer to send the output data
**                      struct EpsonOptions
** Return value:        None
**
*****************************************************************************/
void printHeaderRow(FILE *fp, struct EpsonOptions options) {
  fprintf(fp, "\r\nsample[dec]");
  if (options.flag_out) {
    fprintf(fp, ", ndflags[hex]");
  }

  if (options.temp_out) {
    fprintf(fp, ", tempC[degC]");
  }

  if (options.gyro_out) {
    fprintf(fp, ", gx[dps], gy[dps], gz[dps]");
  }

  if (options.accel_out) {
    fprintf(fp, ", ax[dps], ay[dps], az[dps]");
  }

  if (options.gyro_delta_out) {
    fprintf(fp, ", dax[deg], day[deg], daz[deg]");
  }

  if (options.accel_delta_out) {
    fprintf(fp, ", dvx[m/s], dvy[m/s], dvz[m/s]");
  }

  if (options.qtn_out) {
    fprintf(fp, ", qtn0, qtn1, qtn2, qtn3");
  }

  if (options.atti_out) {
    fprintf(fp, ", roll[deg], pitch[deg], yaw[deg]");
  }

  if (options.gpio_out) {
    fprintf(fp, ", gpio[hex]");
  }

  if (options.count_out) {
    fprintf(fp, ", count[dec]");
  }
  fprintf(fp, "\n");
}

/*****************************************************************************
** Function name:       printSensorRow
** Description:         Prints formatted row of sensor data based on how it is
**                      configured.
** Parameters:          fp - file pointer to send data
**                      options - The struct describing how the IMU is
**                                configured.
**                      epson_data - The struct that is filled with data from
**                                   the IMU.
**                      sample_count - Typically and incrementing index for each
**                                     sample
** Return value:        none
** Notes:
******************************************************************************/
void printSensorRow(FILE *fp, struct EpsonOptions options,
                    struct EpsonData *epson_data, int sample_count) {
  fprintf(fp, "%08d", sample_count);
  if (options.flag_out) {
    fprintf(fp, ", %04x", epson_data->ndflags);
  }

  if (options.temp_out) {
    fprintf(fp, ", %8.3f", epson_data->temperature);
  }

  if (options.gyro_out) {
    fprintf(fp, ", %8.5f, %8.5f, %8.5f", epson_data->gyro_x * 180.0 / 3.14159,
            epson_data->gyro_y * 180.0 / 3.14159,
            epson_data->gyro_z * 180.0 / 3.14159);
  }

  if (options.accel_out) {
    fprintf(fp, ", %8.5f, %8.5f, %8.5f", epson_data->accel_x * 1000 / 9.80665,
            epson_data->accel_y * 1000 / 9.80665,
            epson_data->accel_z * 1000 / 9.80665);
  }

  if (options.gyro_delta_out) {
    fprintf(fp, ", %8.5f, %8.5f, %8.5f", epson_data->gyro_delta_x,
            epson_data->gyro_delta_y, epson_data->gyro_delta_z);
  }

  if (options.accel_delta_out) {
    fprintf(fp, ", %8.5f, %8.5f, %8.5f", epson_data->accel_delta_x,
            epson_data->accel_delta_y, epson_data->accel_delta_z);
  }

  if (options.qtn_out) {
    fprintf(fp, ", %8.8f, %8.8f, %8.8f, %8.8f", epson_data->qtn0,
            epson_data->qtn1, epson_data->qtn2, epson_data->qtn3);
  }

  if (options.atti_out) {
    fprintf(fp, ", %8.3f, %8.3f, %8.3f", epson_data->roll * 180.0 / 3.14159,
            epson_data->pitch * 180.0 / 3.14159,
            epson_data->yaw * 180.0 / 3.14159);
  }

  if (options.gpio_out) {
    fprintf(fp, ", %04x", epson_data->gpio);
  }

  if (options.count_out) {
    fprintf(fp, ", %09d", epson_data->count);
  }
  fprintf(fp, "\n");
}
