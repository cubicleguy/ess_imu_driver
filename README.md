# README for Epson IMU Driver for ROS1

<!---toc start-->

- [README for Epson IMU Driver for ROS1](#readme-for-epson-imu-driver-for-ros1)
  - [What is this repository for?](#what-is-this-repository-for)
  - [What kind of hardware or software will I likely need?](#what-kind-of-hardware-or-software-will-i-likely-need)
    - [For the UART Interface:](#for-the-uart-interface)
    - [For the SPI Interface:](#for-the-spi-interface)
  - [How do I use the driver?](#how-do-i-use-the-driver)
  - [How do I use the driver if usleep() is not supported for time delays?](#how-do-i-use-the-driver-if-usleep-is-not-supported-for-time-delays)
  - [How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?](#how-do-i-use-the-driver-with-gpios-to-control-imu-reset-drdy-ext-pins)
  - [How do I build, install, run this ROS1 package?](#how-do-i-build-install-run-this-ros1-package)
    - [Example console output of *catkin_make* build for G366PDG0:](#example-console-output-of-catkin_make-build-for-g366pdg0)
    - [Example console output of launching ROS node for G366PDG0:](#example-console-output-of-launching-ros-node-for-g366pdg0)
  - [What does this ROS IMU Node Publish as messages?](#what-does-this-ros-imu-node-publish-as-messages)
    - [Without Quaternion Output](#without-quaternion-output)
    - [With Quaternion Output](#with-quaternion-output)
  - [Why am I seeing incorrect ROS timestamps or high latencies or slower than expected IMU data rates?](#why-am-i-seeing-incorrect-ros-timestamps-or-high-latencies-or-slower-than-expected-imu-data-rates)
    - [When using USB-UART bridges](#when-using-usb-uart-bridges)
      - [Modifying *latency_timer* by udev mechanism](#modifying-latency_timer-by-udev-mechanism)
      - [Modifying *latency_timer* by sysfs mechanism](#modifying-latency_timer-by-sysfs-mechanism)
      - [Modifying *low_latency* flag using *setserial* utility](#modifying-low_latency-flag-using-setserial-utility)
    - [When using the SPI interface](#when-using-the-spi-interface)
  - [Package Contents](#package-contents)
  - [License](#license)
  - [References](#references)

<!---toc end-->

## What is this repository for?

- This code provides communication between Epson IMU and ROS using the either the UART or SPI interface.
- For the UART connection, this code uses the standard Unix Terminal I/O library (termios) for communicating either direct or by USB-serial converter such as FTDI USB-UART bridge ICs.
- For the SPI connection, this code uses the [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/) library for accessing GPIO and SPI functions on the Raspberry Pi platform running Ubuntu Linux distro.
- The src/epson_imu_uart_ros_node.cpp is the ROS C++ wrapper used to communicate with ROS using the UART interface.
- The src/epson_imu_spi_ros_node.cpp is the ROS C++ wrapper used to communicate with ROS using the SPI interface.
- The other source files in src/ are based on the Linux C driver originally released by Epson:
  [Epson IMU Linux User-space Driver Example](https://vdc.epson.com/imu-products/imu-inertial-measurement-units)
- Information about ROS, ROS packages, and tutorials can be found: [ROS.org](https://www.ros.org)

## What kind of hardware or software will I likely need?

- [Epson IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/) (G320/G330/G354/G364/G365/G366/G370/V340)
- ROS Noetic, Melodic, Lunar, Kinetic, Indigo (via download) [ROS.org](https://www.ros.org)
- Refer to the ROS installation guide [ROS.org](https://wiki.ros.org/ROS/Installation)
- This software was developed and tested on the following:

```
  ROS1:        Noetic
  Description: Ubuntu 20.04.4 LTS
  Codename:    Focal
```

### For the UART Interface:

- Epson USB evaluation board or equivalent FTDI USB-Serial interface to connect the IMU to ROS host (tty/serial) [See M-G32EV041](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
- Or alternatively, a direct connection from the IMU to the ROS platform that has 3.3V CMOS compatible UART interface [See M-G32EV031](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/).

### For the SPI Interface:

- **NOTE:** This software is intended for use on an embedded Linux host system with a SPI interface (SCLK, MISO, MOSI) and 3 GPIOs (CS#, RST#, DRDY).
  - For Raspberry Pi, the SPI interface should already be enabled using *raspi-config* or equivalent.
  - This code uses a separate GPIO to manually toggle CS# chipselect instead of the chipselect assigned to the HW SPI interface.
    - The assigned chipselect of the HW SPI interface should also work, but has not been thoroughly tested.
- Epson Breakout evaluation board or some equivalent to connect to ROS host (SPI & GPIOs) [See M-G32EV031](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)

## How do I use the driver?

- This code assumes that the user is familiar with building ROS packages using the catkin build process.

- **NOTE:** This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS driver.

- Please refer to the ROS.org website for more detailed instructions on configuring the ROS environment & the ROS package build process. [ROS.org](https://wiki.ros.org/ROS/Tutorials)

- **NOTE:** At bare minimum, you must re-build the package with *catkin_make* after modifying the *CMakeLists.txt* such as changing the following

  - IMU model
  - serial interface type (UART or SPI)
  - host platform type (RPI or NONE=PC)

- If the IMU model is unchanged, then subsequent changes to IMU settings can be done by instead editing the IMU model specific launch file located in the launch/ folder

- The IMU model specific launch file should be used in conjunction with the same catkin-built executable of the same matching the IMU model.

- **NOTE:** Do not just switch launch files without modifying the CMakeLists.txt & rebuilding the executable to match the IMU model. Do not mix IMU model launch files without the matching IMU model colcon built binaries.

## How do I use the driver if usleep() is not supported for time delays?

- **NOTE:** In the hcl_linux.c or hcl_rpi.c, there are wrapper functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
- On embedded Linux platforms, the user may need modify and redirect to platform specific delay routines if usleep() is not supported.
- For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
- If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

- When connecting the IMU using the UART interface, the use of GPIO pins for connecting to the IMU RESET#, EXT, or DRDY is purely optional, and mainly intended for use with embedded Linux platforms (such as RapsberryPi).
- When connecting the IMU using the SPI interface, the use of GPIO pins for connecting to the IMU DRDY is mandatory (RESET# is recommended, EXT is optional).
- The user can choose to connect the IMU CS# pin directly with a designated host GPIO output or connect to the designated chipselect of the SPI interface (Default is SPI_CE0 pin on the RapsberryPi)
- When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization for better robustness.
- This code is structured for the user to easily redirect GPIO control to low-level hardware GPIO function calls for ease of implementation.
- There is no standard method to implement GPIO connections on embedded Linux platform, but the following files are examples of typical changes:

```
  src/hcl_linux.c
  src/hcl_gpio.c
  src/hcl_gpio.h
```

- Typically, an external library needs to be invoked to initialize & enable GPIO HW functions on the user's embedded platform.

- This typically requires changes to hcl_linux.c (Use hcl_rpi.c as a template)

  - add #include to external library near the top of hcl_linux.c
  - add the initialization call inside the seInit() function in hcl_linux.c

For example on an Raspberry Pi, the following changes can be made to hcl_linux.c:

```
...

  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>                                                                // <== Added external library

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }

...
```

- Typically, the GPIO pins need to be assigned according to pin numbering specific to the embedded HW platform.
- This typically requires changes to hcl_gpio.h

**NOTE:** When using the SPI interface, if you will not control the CS# by host GPIO, then connect IMU CS# pin to the RPI SPI0_CE0, P1_24.

For example on an Raspberry Pi, the following changes to hcl_gpio.h with the following pin mapping:

```
    Epson IMU                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
```

```
...

  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);

  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);

  #define RPI_GPIO_P1_15              22                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added

  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
...
```

- Typically, the external library will have GPIO pin control functions such as set_output(), set_input(), set(), reset(), read_pin_level(), etc...

- This requires changes to hcl_gpio.c (Use hcl_gpio_rpi.c as a template).

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.
  - For example on an Raspberry Pi, the following changes to hcl_gpio.c:

```
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library

...

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }

...

  int gpioRelease(void)
  {
    return OK;
  }

...

  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  ...

  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  ...

  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  ...
```

## How do I build, install, run this ROS1 package?

The Epson IMU ROS1 driver is designed to be built in the standard ROS catkin build environment.
Refer to the ROS1 Tutorials for more info: [ROS1 Tutorial](https://wiki.ros.org/ROS/Tutorials)

For more information on ROS & catkin setup refer to:
[Installing and Configuring ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

1. Place this package (including folders) into a new folder within your catkin workspace "src" folder.
   For example, we recommend using the folder name "ess_imu_driver"

```
   <catkin_workspace>/src/ess_imu_driver/
```

2. Modify the *CMakeLists.txt* to select the desired Epson IMU model, serial interface type, platform type that matches the attached ROS system.
   Refer to the comment lines inside the *CMakeLists.txt* for additional info.
   **NOTE:** You *MUST* re-build using *catkin_make* when changing IMU models or after any changes in the *CMakeLists.txt*

3. From the catkin workspace folder run *catkin_make* to build all ROS1 packages located in the \<catkin_workspace>/src/ folder.

**NOTE:** You *MUST* re-run the above *catkin_make* command to rebuild the driver after making any changes to the *CMakeLists.txt*, .c or .cpp or .h source files.

**NOTE:** It is recommended to change IMU settings by editing the parameters in the approriate launch file, wherever possible, instead of modifying the .c or .cpp source files directly.

```
   <catkin_workspace>/catkin_make
```

4. Reload the current ROS environment variables that may have changed after the catkin build process.

```
   From the <catkin_workspace>: source devel/setup.bash
```

5. Modify the appropriate launch file for the IMU model in the launch/ folder to set your desired IMU configure parameter options at runtime:

| Launch Parameter | Comment                                                                                                                                                                     |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| port             | **NOTE:** Only applicable when using UART interface to specify the port name, otherwise ignored ie. "/dev/ttyUSB0" or "/dev/ttyXXX"                                         |
| ext_sel          | specifies the function of the GPIO2 pin (GPIO2, External Trigger, External Counter Reset). **NOTE:** Must be set to External Counter Reset when time_correction is enabled. |
| ext_pol          | specifies the polarity of the GPIO2 pin when *ext_sel* is External Trigger or External Counter Reset **NOTE:** Only applicable to G320/G354/G364/V340 otherwise ignored.    |
| drdy_on          | specifies to enable DRDY output function on GPIO1. **NOTE:** Must be enabled when using SPI interface                                                                       |
| drdy_pol         | specifies the polarity of the DRDY output pin when enabled. **NOTE:** Must be set to 1 (active high) when using SPI interface                                               |
| dout_rate        | specifies the IMU output data rate                                                                                                                                          |
| filter_sel       | specifies the IMU filter setting                                                                                                                                            |
| flag_out         | specifies to enable or disable ND_FLAG status in IMU output data (not used by ROS)                                                                                          |
| temp_out         | specifies to enable or disable TempC sensor in IMU output data (**must be enabled**)                                                                                        |
| gyro_out         | specifies to enable or disable Gyro sensor in IMU output data (**must be enabled**)                                                                                         |
| accel_out        | specifies to enable or disable Accl sensor in IMU output data (**must be enabled**)                                                                                         |
| gyro_delta_out   | specifies to enable or disable DeltaAngle in IMU output data (not used by ROS)                                                                                              |
| accel_delta_out  | specifies to enable or disable DeltaVelocity in IMU output data (not used by ROS)                                                                                           |
| qtn_out          | specifies to enable or disable Quaternion in IMU output data (**only supported for G330/G365/G366, otherwise ignored**)                                                     |
| atti_out         | specifies to enable or disable Attitude in IMU output data (not used by ROS)                                                                                                |
| gpio_out         | specifies to enable or disable GPIO in IMU output data (not used by ROS)                                                                                                    |
| count_out        | specifies to enable or disable counter in IMU output data (**must be enabled when time_correction is enabled**)                                                             |
| checksum_out     | specifies to enable or disable checksum in IMU output data (not used by ROS, but checksum errors are detected when enabled)                                                 |
| temp_bit         | specifies to 16 or 32 bit resolution in TempC output data (**recommend setting to 32-bit**)                                                                                 |
| gyro_bit         | specifies to 16 or 32 bit resolution in Gyro output data (**recommend setting to 32-bit**)                                                                                  |
| accel_bit        | specifies to 16 or 32 bit resolution in Accl output data (**recommend setting to 32-bit**)                                                                                  |
| gyro_delta_bit   | specifies to 16 or 32 bit resolution in DeltaAngle output data (not used by ROS)                                                                                            |
| accel_delta_bit  | specifies to 16 or 32 bit resolution in DeltaVelocity output data (not used by ROS)                                                                                         |
| qtn_bit          | specifies to 16 or 32 bit resolution in Quaternion output data (**used for orientation field, only supported for G330/G365/G366, otherwise ignored**)                       |
| invert_xgyro     | specifies to reverse polarity of this sensor axis                                                                                                                           |
| invert_ygyro     | specifies to reverse polarity of this sensor axis                                                                                                                           |
| invert_zgyro     | specifies to reverse polarity of this sensor axis                                                                                                                           |
| invert_xaccel    | specifies to reverse polarity of this sensor axis                                                                                                                           |
| invert_yaccel    | specifies to reverse polarity of this sensor axis                                                                                                                           |
| invert_zaccel    | specifies to reverse polarity of this sensor axis                                                                                                                           |
| atti_mode        | specifies the attitude mode as 0=inclination or 1=euler (not used by ROS, **only supported for G330/G365/G366**)                                                            |
| atti_profile     | specifies the attitude motion profile (**used for orientation field, only supported for G330/G365/G366, otherwise ignored**)                                                |
| time_correction  | enables time correction function using External Counter Reset function & external 1PPS connected to IMU GPIO2/EXT pin. **NOTE:** Must set count_out=1 & ext_sel=1           |

**NOTE:** The ROS launch file passes IMU configuration settings to the IMU at runtime. Therefore, rebuilding with *catkin_make* is not required.

6. The Epson IMU ROS1 driver is started with the appropriate launch file (located in launch/) from console.
   All parameters are described in the in-line comments of the launch file.
   The launch file contains parameters for configuring IMU settings at runtime.

| Launch File               | Description                                                                                                                  |
| ------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| g320_g354_g364.launch     | For G320PDG0/G354PDH0/G364DC0/G364PDCA, outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)   |
| g330_g365_g366.launch     | For G330PDG0/G365PDx1/G366PDG0, outputs to ROS topic imu/data (**gyro, accel data, including quaternion orientation**)       |
| g330_g365_g366_raw.launch | For G330PDG0/G365PDx1/G366PDG0, outputs to ROS topic imu/data_raw (**gyro, accel data, but no quaternion orientation**)      |
| g370.launch               | For G370PDF1/G370PDS0/G370PDG0/G370PDT0 , outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**) |
| v340.launch               | For V340PDD0, outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)                             |

For example, for the Epson G366PDG0 IMU:

```
   <catkin_workspace>/roslaunch ess_imu_driver g330_g365_g366.launch
```

### Example console output of *catkin_make* build for G366PDG0:

```
Base path: /home/user/catkin_ws
Source space: /home/user/catkin_ws/src
Build space: /home/user/catkin_ws/build
Devel space: /home/user/catkin_ws/devel
Install space: /home/user/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/user/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/user/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/user/catkin_ws/devel;/opt/ros/noetic
-- This workspace overlays: /home/user/catkin_ws/devel;/opt/ros/noetic
-- Found PythonInterp: /usr/bin/python3 (found suitable version "3.8.10", minimum required is "3")
-- Using PYTHON_EXECUTABLE: /usr/bin/python3
-- Using Debian Python package layout
-- Using empy: /usr/lib/python3/dist-packages/em.py
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/user/catkin_ws/build/test_results
-- Forcing gtest/gmock from source, though one was otherwise available.
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python3 (found version "3.8.10")
-- Using Python nosetests: /usr/bin/nosetests3
-- catkin 0.8.10
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 1 packages in topological order:
-- ~~  - ess_imu_driver
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'ess_imu_driver'
-- ==> add_subdirectory(ess_imu_driver)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Building for IMU Model: G366PDG0
---- Building for platform: RPI
---- Building for interface: UART
-- Configuring done
-- Generating done
-- Build files have been written to: /home/user/catkin_ws/build
####
#### Running command: "make -j4 -l4" in "/home/user/catkin_ws/build"
####
Scanning dependencies of target ess_imu_driver_lib
[ 22%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/hcl_rpi.c.o
[ 22%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/hcl_gpio_rpi.c.o
[ 33%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/hcl_uart.c.o
[ 44%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/sensor_epsonCommon.c.o
[ 55%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/sensor_epsonUart.c.o
[ 66%] Building C object ess_imu_driver/CMakeFiles/ess_imu_driver_lib.dir/src/sensor_epsonG330_G366.c.o
[ 77%] Linking C shared library /home/user/catkin_ws/devel/lib/libess_imu_driver_lib.so
[ 77%] Built target ess_imu_driver_lib
Scanning dependencies of target ess_imu_driver_node
[ 88%] Building CXX object ess_imu_driver/CMakeFiles/ess_imu_driver_node.dir/src/epson_imu_uart_ros_node.cpp.o
[100%] Linking CXX executable /home/user/catkin_ws/devel/lib/ess_imu_driver/ess_imu_driver_node
[100%] Built target ess_imu_driver_node
```

### Example console output of launching ROS node for G366PDG0:

```
user@VDC-RPI3B-PLUS:~/catkin_ws$ roslaunch ess_imu_driver g330_g365_g366.launch
... logging to /home/user/.ros/log/91f019a6-6ee1-11ee-a745-e523098db919/roslaunch-VDC-RPI3B-PLUS-3208.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://VDC-RPI3B-PLUS:36439/

SUMMARY
========

PARAMETERS
 * /ess_imu_driver_node/accel_bit: 1
 * /ess_imu_driver_node/accel_delta_bit: 1
 * /ess_imu_driver_node/accel_delta_out: 0
 * /ess_imu_driver_node/accel_out: 1
 * /ess_imu_driver_node/atti_bit: 1
 * /ess_imu_driver_node/atti_conv: 0
 * /ess_imu_driver_node/atti_mode: 1
 * /ess_imu_driver_node/atti_out: 0
 * /ess_imu_driver_node/atti_profile: 0
 * /ess_imu_driver_node/checksum_out: 1
 * /ess_imu_driver_node/count_out: 1
 * /ess_imu_driver_node/dout_rate: 4
 * /ess_imu_driver_node/drdy_on: 1
 * /ess_imu_driver_node/drdy_pol: 1
 * /ess_imu_driver_node/ext_pol: 0
 * /ess_imu_driver_node/ext_sel: 1
 * /ess_imu_driver_node/filter_sel: 5
 * /ess_imu_driver_node/flag_out: 1
 * /ess_imu_driver_node/gpio_out: 0
 * /ess_imu_driver_node/gyro_bit: 1
 * /ess_imu_driver_node/gyro_delta_bit: 1
 * /ess_imu_driver_node/gyro_delta_out: 0
 * /ess_imu_driver_node/gyro_out: 1
 * /ess_imu_driver_node/invert_xaccel: 0
 * /ess_imu_driver_node/invert_xgyro: 0
 * /ess_imu_driver_node/invert_yaccel: 0
 * /ess_imu_driver_node/invert_ygyro: 0
 * /ess_imu_driver_node/invert_zaccel: 0
 * /ess_imu_driver_node/invert_zgyro: 0
 * /ess_imu_driver_node/port: /dev/ttyUSB0
 * /ess_imu_driver_node/qtn_bit: 1
 * /ess_imu_driver_node/qtn_out: 1
 * /ess_imu_driver_node/temp_bit: 1
 * /ess_imu_driver_node/temp_out: 1
 * /ess_imu_driver_node/time_correction: 0
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
  /
    ess_imu_driver_node (ess_imu_driver/ess_imu_driver_node)

auto-starting new master
process[master]: started with pid [3220]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 91f019a6-6ee1-11ee-a745-e523098db919
process[rosout-1]: started with pid [3233]
started core service [/rosout]
process[ess_imu_driver_node-2]: started with pid [3240]
[ INFO] [1697762563.301904404]: Initializing HCL layer...

Initializing libraries......done.[ INFO] [1697762563.309323546]: Initializing GPIO interface...
[ INFO] [1697762563.309658285]: Initializing UART interface...
Attempting to open port.../dev/ttyUSB0
...sensorDummyWrite.[ INFO] [1697762563.513332256]: Checking sensor NOT_READY status...
...done.[ INFO] [1697762564.343224343]: Initializing Sensor...
[ INFO] [1697762564.356152947]: Epson IMU initialized.
[ INFO] [1697762564.356452685]: Compiled for:   G366PDG0
[ INFO] [1697762564.356651487]: Reading device info...
[ INFO] [1697762564.363174799]: PRODUCT ID:     G370PDG0
[ INFO] [1697762564.369008999]: SERIAL ID:      M0000012
...Sensor start.[ INFO] [1697762564.382557757]: Quaternion Output: Native.

```

## What does this ROS IMU Node Publish as messages?

The Epson IMU ROS driver will publish messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

- For IMU models such as G320/G354/G364/G370/V340, the IMU messages **will only update the fields for angular rate (gyro) and linear acceleration (accel) data**.
- For IMU models G330/G365/G366, it depends on if the IMU attitude function with quaternion output enabled or not:
  - IMU messages will only update *angular_velocity* (gyro) and *linear_acceleration* (accel) fields when quaternion output is **disabled**, i.e. launch file *g330_g365_g366_raw.launch*
  - IMU messages will update update *angular_velocity* (gyro), *linear_acceleration* (accel), and *orientation* field using the internal extended Kalman Filter when the quaternion output is **enabled**, ie. launch file *g330_g365_g366.launch*
- Temperature sensor data from the IMU is also published on topic epson_tempc/ and will be remapped by the launch file to */imu/tempc*

### Without Quaternion Output

For non-quaternion output models, the ROS driver will publish to the following ROS topic:

```
/epson_imu/data_raw <-- orientation field will not contain valid data & should be ignored
```

**NOTE:** The launch file will remap the ROS message to publish on */imu/data_raw*

```
---
header:
  seq: 34983
  stamp:
    secs: 1601590579
    nsecs: 771273325
  frame_id: "imu_link"
orientation:
  x: 2.4786295434e+33
  y: 1.1713334935e+38
  z: 1.17130631507e+38
  w: 1.17130956026e+38
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.00435450254008
  y: 0.000734272529371
  z: -9.40820464166e-05
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.730921983719
  y: -1.54766368866
  z: 9.72711181641
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```

### With Quaternion Output

- For quaternion output models, the ROS driver will publish to the following ROS topic:

```
/epson_imu/data <-- orientation, angular_velocity, linear_acceleration fields will be updating
```

**NOTE:** The launch file will remap the ROS message to publish on */imu/data*

```
---
header:
  seq: 10608
  stamp:
    secs: 1601575987
    nsecs: 673387357
  frame_id: "imu_link"
orientation:
  x: -0.0801454782486
  y: 0.0367396138608
  z: 0.00587898213416
  w: 0.996088504791
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: -0.00118702522013
  y: 0.000320095219649
  z: -0.00014466587163
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.727666378021
  y: -1.5646469593
  z: 9.69056034088
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```

## Why am I seeing incorrect ROS timestamps or high latencies or slower than expected IMU data rates?

- By default, this software publishes the timestamp using ROS/Unix time for ROS IMU messages at the instant new IMU data is detected.
- Latencies from system overhead, system loading, and buffering will cause inaccuracies in this ROS timestamp.
- To improve timestamp accuracy, the user should consider adding GNSS receiver with 1PPS output to the ROS host system:
  - Consider setting up [chrony](https://chrony-project.org/examples.html) and [gpsd](https://gpsd.io/) with the GNSS receiver's 1PPS signal connected to the host system to improve host system clock accuracy
  - Consider also connecting the GNSS receiver's 1PPS signal to the Epson IMU GPIO2/EXT with *time_correction* enabled in the launch file to improve timstamp accuracy of the IMU sensor messages

### When using USB-UART bridges

- If your connection between the Epson IMU UART and the Linux host is by FTDI,
  the *latency_timer* setting in the FTDI driver may be large i.e. typically 16 (msec).
- There are 3 methods listed below to reduce the impact of this latency.

#### Modifying *latency_timer* by udev mechanism

- [udev](https://wiki.debian.org/udev) is a device manager for Linux that can dynamically create and remove devices in *userspace* and run commands when new devices appear or other events
- Create a udev rule to automatically set the *latency_timer* to 1 when an FTDI USB-UART device is plugged in
- For example, here is a text file named *99-ftdi_sio.rules* that can be put in the */etc/udev/rules.d* directory

```
SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
```

**NOTE:** Requires root (sudo) access to create or copy file in */etc/udev/rules.d*
**NOTE:** This is the more robust method because it automatically sets when device is plugged in, but affects ALL FTDI USB-UART devices on the system

#### Modifying *latency_timer* by sysfs mechanism

- The example below reads the *latency_timer* setting for */dev/ttyUSB0* which returns 16msec.
- Then, it sets the *latency_timer* to 1msec, and confirms it by readback.

**NOTE: May require root (sudo su) access on your system to modify.**

```
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
16
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
```

#### Modifying *low_latency* flag using *setserial* utility

- The example below sets the *low_latency* flag for /dev/ttyUSB0.
- This will have the same effect as setting the *latency_timer* to 1msec.
- This can be confirmed by running the *setserial* command again.

```
user@user:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0

user@user:~$ setserial /dev/ttyUSB0 low_latency

user@user:~$ setserial /dev/ttyUSB0
/dev/ttyUSB0, UART: unknown, Port: 0x0000, IRQ: 0, Flags: low_latency
```

### When using the SPI interface

- Issues with SPI interface will largely depend on your host system processing load and capabilities.
- If your ROS platform is running too many ROS node packages or simply too slow it may not react fast enough to detect the rising edge of the IMU DRDY signal and
  process the IMU sampling data.
- Try modifying the *dout_rate* and *filter_sel* to the slowest setting that can meet your ROS system requirements.
- Monitor the DRDY signal on the IMU with a oscilloscope to verify the systems processing reliability by monitoring the stability of the IMU DRDY signal and seeing that it matches the expected *dout_rate*.
- Try set the SPI clock rate to maximum of 1MHz (1000000), in the *\_node.cpp* for the spiInit() function call:

```
...

  ROS_INFO("Initializing SPI interface...");
  // The max SPI clock rate is 1MHz for current model Epson IMUs
  if (!spiInit(SPI_MODE3, 1000000)) {
    ROS_ERROR("Error: could not initialize SPI interface. Exiting...");
    gpioRelease();
    seRelease();
    return false;
  }


...
```

## Package Contents

The Epson IMU ROS1 driver-related sub-folders & root files are:

```
   launch/        <== various example launch files for Epson IMU models
   src/           <== source code for ROS node C++ wrapper, IMU Linux C driver, and additional README_src.md specifically for building and using the IMU Linux C driver as stand-alone (without ROS support)
   CHANGELOG.rst  <== summarizes revision changes
   CMakeLists.txt <== cmake build script for catkin_make
   LICENSE.txt    <== description of the applicable licenses
   package.xml    <== catkin package description
   README.md      <== general README
```

## License

Refer to *LICENSE.txt* in this package

## References

1. https://index.ros.org
