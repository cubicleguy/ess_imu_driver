//==============================================================================
//
// 	epson_imu_uart_driver_node.cpp
//     - ROS node for Epson IMU sensor evaluation
//     - This program initializes the Epson IMU and publishes ROS messages in
//       ROS topic /epson_imu as convention per [REP 145]
//       (http://www.ros.org/reps/rep-0145.html).
//     - If the IMU model supports quaternion output (currently supported only
//     by
//       G330/G365/G366) orientation fields are updated in published topic
//       /epson_imu/data
//     - If the IMU model does not support quaternion output, then
//       orientation fields do not update in published
//       topic /epson_imu/data_raw
//
//  [This software is BSD-3
//  licensed.](http://opensource.org/licenses/BSD-3-Clause)
//
//  Original Code Development:
//  Copyright (c) 2019, Carnegie Mellon University. All rights reserved.
//
//  Additional Code contributed:
//  Copyright (c) 2019, 2023, Seiko Epson Corp. All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//  THE POSSIBILITY OF SUCH DAMAGE.
//
//==============================================================================

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Temperature.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>
#include <string>
#include <termios.h>

#include "hcl.h"
#include "hcl_gpio.h"
#include "hcl_uart.h"
#include "sensor_epsonCommon.h"

using namespace std;

string serial_port;

//=========================================================================
//------------------------ IMU Initialization -----------------------------
//=========================================================================

bool init_imu(const struct EpsonOptions& options) {
  ROS_INFO("Initializing HCL layer...");
  if (!seInit()) {
    ROS_ERROR(
        "Error: could not initialize the Seiko Epson HCL layer. Exiting...");
    return false;
  }

  ROS_INFO("Initializing GPIO interface...");
  if (!gpioInit()) {
    ROS_ERROR("Error: could not initialize the GPIO layer. Exiting...");
    seRelease();
    return false;
  }

  ROS_INFO("Initializing UART interface...");
  // The baudrate value should be set the the same setting as currently
  // flashed value in the IMU UART_CTRL BAUD_RATE register
  if (!uartInit(serial_port.c_str(), BAUD_460800)) {
    ROS_ERROR("Error: could not initialize UART interface. Exiting...");
    gpioRelease();
    seRelease();
    return false;
  }

  // This is required for NVIDIA Jetson TK1 but root cause is still TBD.
  // Current assumption is that when the serial port is first opened, there are
  // corrupted characters sent on first serial messages.
  // So sending dummy command may work around this condition
  // Investigation TBD
  sensorDummyWrite();

  ROS_INFO("Checking sensor NOT_READY status...");
  if (!sensorPowerOn()) {
    ROS_ERROR("Error: failed to power on Sensor. Exiting...");
    uartRelease();
    gpioRelease();
    seRelease();
    return false;
  }
  printf("...done.");

  ROS_INFO("Initializing Sensor...");
  if (!sensorInitOptions(options)) {
    ROS_ERROR("Error: could not initialize Epson Sensor. Exiting...");
    uartRelease();
    gpioRelease();
    seRelease();
    return false;
  }

  ROS_INFO("Epson IMU initialized.");
  return true;
}

//=========================================================================
//------------------------ IMU Identify PROD_ID & SER_NUM -----------------
//=========================================================================

std::string get_prod_id() {
  unsigned short prod_id1 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID1, false);
  unsigned short prod_id2 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID2, false);
  unsigned short prod_id3 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID3, false);
  unsigned short prod_id4 = registerRead16(CMD_WINDOW1, ADDR_PROD_ID4, false);

  char myarray[] = {
      static_cast<char>(prod_id1), static_cast<char>(prod_id1 >> 8),
      static_cast<char>(prod_id2), static_cast<char>(prod_id2 >> 8),
      static_cast<char>(prod_id3), static_cast<char>(prod_id3 >> 8),
      static_cast<char>(prod_id4), static_cast<char>(prod_id4 >> 8)};
  std::string prod_id(myarray);
  return prod_id;
}

std::string get_serial_id() {
  unsigned short ser_num1 =
      registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM1, false);
  unsigned short ser_num2 =
      registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM2, false);
  unsigned short ser_num3 =
      registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM3, false);
  unsigned short ser_num4 =
      registerRead16(CMD_WINDOW1, ADDR_SERIAL_NUM4, false);

  char myarray[] = {
      static_cast<char>(ser_num1), static_cast<char>(ser_num1 >> 8),
      static_cast<char>(ser_num2), static_cast<char>(ser_num2 >> 8),
      static_cast<char>(ser_num3), static_cast<char>(ser_num3 >> 8),
      static_cast<char>(ser_num4), static_cast<char>(ser_num4 >> 8)};
  std::string ser_num(myarray);
  return ser_num;
}

void identify_build() { ROS_INFO_STREAM("Compiled for:\t" << BUILD_FOR); }

void identify_device() {
  ROS_INFO("Reading device info...");
  ROS_INFO("PRODUCT ID:\t%s", get_prod_id().c_str());
  ROS_INFO("SERIAL ID:\t%s", get_serial_id().c_str());
}

//=========================================================================
//----------------------- Timestamp Correction ----------------------------
//=========================================================================

class TimeCorrection {
 private:
  int32_t MAX_COUNT;
  int32_t ALMOST_ROLLOVER;
  int32_t ONE_SEC_NSEC;
  int32_t HALF_SEC_NSEC;

  int32_t count_corrected;
  int32_t count_corrected_old;
  int32_t count_old;
  int32_t count_diff;
  int32_t time_sec_current;
  int32_t time_sec_old;
  int32_t time_nsec_current;
  bool rollover;
  bool flag_imu_lead;

 public:
  TimeCorrection();
  ros::Time get_stamp(int);
};

TimeCorrection::TimeCorrection() {
  // ALMOST_ROLLOVER value depends on system processing speed & overhead latency
  // When accurate 1PPS signal is connected to GPIO2/EXT, counter reset should
  // never go over ALMOST_ROLLOVER
#if defined G320PDG0 || defined G354PDH0 || defined G364PDC0 || \
    defined G364PDCA || defined V340PDD0
  // Counter freq = 46875Hz, Max Count = 65535/46875 * 1e9
  MAX_COUNT = 1398080000;
  ALMOST_ROLLOVER = 1340000000;
#else
  // Counter freq = 62500Hz, Max Count = 65535/62500 * 1e9
  MAX_COUNT = 1048560000;
  ALMOST_ROLLOVER = 1010000000;
#endif
  ONE_SEC_NSEC = 1000000000;
  HALF_SEC_NSEC = 500000000;

  count_corrected = 0;
  count_corrected_old = 0;
  count_old = 0;
  count_diff = 0;
  time_sec_current = 0;
  time_sec_old = 0;
  time_nsec_current = 0;
  rollover = false;
  flag_imu_lead = false;
}

//=========================================================================
// TimeCorrection::get_stamp
//
// Returns the timestamp based on time offset from most recent 1PPS signal.
// Epson IMU has a free-running upcounter that resets on active 1PPS signal.
// Counter value is embedded in the sensor data at the time of sampling.
// Time stamp is corrected based on reset counter retrieved from embedded
// sensor data.
//
// This assumes that the ROS time is sync'ed to 1PPS pulse sent to
// Epson IMU GPIO2_EXT pin and the IMU Counter Reset feature is enabled
// and the ROS latency from IMU sample to calling ros::Time::now() is
// less than 0.020 seconds, otherwise the timestamp returned may be
// unreliable.
// The IMU count is already converted to nsecs units (should always be
// less than ONE_SEC_NSEC (1e9)
//=========================================================================
ros::Time TimeCorrection::get_stamp(int count) {
  time_sec_current = ros::Time::now().toSec();
  time_nsec_current = ros::Time::now().nsec;
  std::cout.precision(20);

  count_diff = count - count_old;
  if (count > ALMOST_ROLLOVER) {
    rollover = true;
  }
  if (count_diff < 0) {
    if (rollover) {
      count_diff = count + (MAX_COUNT - count_old);
      ROS_WARN(
          "Warning: time_correction enabled but IMU external counter reset "
          "rollover detected. Is 1PPS connected to IMU GPIO2/EXT pin?");
    } else {
      count_diff = count;
      count_corrected = 0;
    }
    rollover = false;
  }
  count_corrected = (count_corrected + count_diff) % ONE_SEC_NSEC;
  if ((time_sec_current != time_sec_old) && (count_corrected > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current - 1;
  } else if (((count_corrected - count_corrected_old) < 0) &&
             (time_nsec_current > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current + 1;
    flag_imu_lead = true;
  } else if ((flag_imu_lead) && (time_nsec_current > HALF_SEC_NSEC)) {
    time_sec_current = time_sec_current + 1;
  } else {
    flag_imu_lead = false;
  }
  ros::Time time;
  time.nsec = count_corrected;
  time.sec = time_sec_current;
  time_sec_old = time_sec_current;
  count_old = count;
  count_corrected_old = count_corrected;

  return time;
}

//=========================================================================
//------------------------------ Main -------------------------------------
//=========================================================================

int main(int argc, char** argv) {
  ros::init(argc, argv, "epson_imu_uart_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle np("~");

  np.param<string>("port", serial_port, "/dev/ttyUSB0");

  struct EpsonOptions options;
  int time_correction = false;

  // Recommended to change these parameters via .launch file instead of
  // modifying source code below directly
  np.param("ext_sel", options.ext_sel, 1);
  np.param("ext_pol", options.ext_pol, 0);
  np.param("drdy_on", options.drdy_on, 0);
  np.param("drdy_pol", options.drdy_pol, 1);

  np.param("dout_rate", options.dout_rate, 3);
  np.param("filter_sel", options.filter_sel, 5);

  np.param("flag_out", options.flag_out, 1);
  np.param("temp_out", options.temp_out, 1);
  np.param("gyro_out", options.gyro_out, 1);
  np.param("accel_out", options.accel_out, 1);
  np.param("gyro_delta_out", options.gyro_delta_out, 0);
  np.param("accel_delta_out", options.accel_delta_out, 0);
  np.param("qtn_out", options.qtn_out, 0);
  np.param("atti_out", options.atti_out, 0);
  np.param("gpio_out", options.gpio_out, 0);
  np.param("count_out", options.count_out, 1);
  np.param("checksum_out", options.checksum_out, 1);

  np.param("temp_bit", options.temp_bit, 1);
  np.param("gyro_bit", options.gyro_bit, 1);
  np.param("accel_bit", options.accel_bit, 1);
  np.param("gyro_delta_bit", options.gyro_delta_bit, 1);
  np.param("accel_delta_bit", options.accel_delta_bit, 1);
  np.param("qtn_bit", options.qtn_bit, 1);
  np.param("atti_bit", options.atti_bit, 1);

  np.param("invert_xgyro", options.invert_xgyro, 0);
  np.param("invert_ygyro", options.invert_ygyro, 0);
  np.param("invert_zgyro", options.invert_zgyro, 0);
  np.param("invert_xaccel", options.invert_xaccel, 0);
  np.param("invert_yaccel", options.invert_yaccel, 0);
  np.param("invert_zaccel", options.invert_zaccel, 0);

  np.param("dlt_ovf_en", options.dlt_ovf_en, 0);
  np.param("dlt_range_ctrl", options.dlt_range_ctrl, 8);

  np.param("atti_mode", options.atti_mode, 1);
  np.param("atti_conv", options.atti_conv, 0);
  np.param("atti_profile", options.atti_profile, 0);

  np.param("time_correction", time_correction, 0);

  if (!init_imu(options)) return -1;
  identify_build();
  identify_device();

  int result = get_prod_id().compare(BUILD_FOR);
  if (result == 0) {
    ROS_INFO("OK: Build matches detected device");
  } else {
    ROS_ERROR("*** Build *mismatch* with detected device ***");
    ROS_WARN(
        "*** Check the CMakeLists.txt for setting a compatible IMU_MODEL \
                  variable, modify as necessary, and rebuild the driver ***");
  }
  sensorStart();

  struct EpsonData epson_data;
  TimeCorrection tc;

  sensor_msgs::Temperature tempc_msg;
  ros::Publisher tempc_pub =
      nh.advertise<sensor_msgs::Temperature>("epson_tempc", 1);

  sensor_msgs::Imu imu_msg;
  for (int i = 0; i < 9; i++) {
    imu_msg.orientation_covariance[i] = 0;
    imu_msg.angular_velocity_covariance[i] = 0;
    imu_msg.linear_acceleration_covariance[i] = 0;
  }
  imu_msg.orientation_covariance[0] = -1;

  ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("epson_imu", 1);
#ifdef PUB_RPY
  ROS_INFO("Euler Output: SW Conversion from Quaternion.");
  ros::Publisher rpy_pub =
      nh.advertise<geometry_msgs::Vector3Stamped>("epson_imu_rpy", 1);
#endif  // PUB_RPY
  tf2::Quaternion myQuaternion, ang1Quaternion, ang2Quaternion, ang3Quaternion;

#ifdef NATIVE_QUAT
  ROS_INFO("Quaternion Output: Native.");
#else
  ROS_INFO("Quaternion Output: SW Conversion from Euler.");
#endif  // NATIVE_QUAT
  while (ros::ok()) {
    if (sensorDataReadBurstNOptions(options, &epson_data)) {
      imu_msg.header.frame_id = "imu_link";
      if (!time_correction)
        imu_msg.header.stamp = ros::Time::now();
      else
        imu_msg.header.stamp = tc.get_stamp(epson_data.count);
      imu_msg.angular_velocity.x = epson_data.gyro_x;
      imu_msg.angular_velocity.y = epson_data.gyro_y;
      imu_msg.angular_velocity.z = epson_data.gyro_z;
      imu_msg.linear_acceleration.x = epson_data.accel_x;
      imu_msg.linear_acceleration.y = epson_data.accel_y;
      imu_msg.linear_acceleration.z = epson_data.accel_z;
#ifdef NATIVE_QUAT
      myQuaternion[0] = epson_data.qtn1;
      myQuaternion[1] = epson_data.qtn2;
      myQuaternion[2] = epson_data.qtn3;
      myQuaternion[3] = epson_data.qtn0;
#else
      // Initialize quaternion
      myQuaternion.setRPY(0, 0, 0);
      // G365 euler output rotation order is yaw -> roll -> pitch about the
      // rotating frame
      // Create 3 independant quaternion rotations
      // from G365 ANG1(roll), ANG2(pitch), ANG3(yaw)
      ang1Quaternion.setRPY(epson_data.roll, 0, 0);
      ang2Quaternion.setRPY(0, epson_data.pitch, 0);
      ang3Quaternion.setRPY(0, 0, epson_data.yaw);

      // Apply 3 quaternion rotations according G365 euler rotation order
      // (yaw->roll->pitch)
      // about the rotating axis (i.e. post-multiplication of rotation
      // quaternion)
      // and normalize after each
      myQuaternion = myQuaternion * ang3Quaternion;  // yaw
      myQuaternion.normalize();
      myQuaternion = myQuaternion * ang1Quaternion;  // roll
      myQuaternion.normalize();
      myQuaternion = myQuaternion * ang2Quaternion;  // pitch
      myQuaternion.normalize();
#endif  // NATIVE_QUAT
      // Publish quaternion orientation
      imu_msg.orientation.x = myQuaternion[0];
      imu_msg.orientation.y = myQuaternion[1];
      imu_msg.orientation.z = myQuaternion[2];
      imu_msg.orientation.w = myQuaternion[3];
      imu_pub.publish(imu_msg);
      // Publish temperature
      tempc_msg.header = imu_msg.header;
      tempc_msg.temperature = epson_data.temperature;
      tempc_pub.publish(tempc_msg);

#ifdef PUB_RPY
      // Convert the quaternion to euler YPR (rotating frame)
      // which is equivalent to RPY (fixed frame)
      // Then publish roll, pitch, yaw angles
      double roll, pitch, yaw;

      tf2::Matrix3x3(myQuaternion).getRPY(roll, pitch, yaw);
      geometry_msgs::Vector3Stamped rpy;
      rpy.header = imu_msg.header;
      rpy.vector.x = roll;
      rpy.vector.y = pitch;
      rpy.vector.z = yaw;
      rpy_pub.publish(rpy);
#endif  // PUB_RPY
    } else {
      ROS_WARN(
          "Warning: Checksum error or incorrect delimiter bytes in imu_msg "
          "detected");
    }
  }
  sensorStop();
  seDelayMS(1000);
  uartRelease();
  gpioRelease();
  seRelease();

  return 0;
}
