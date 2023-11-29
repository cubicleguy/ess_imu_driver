# README for Epson IMU Linux C Driver using UART or SPI interface

<!---toc start-->

- [README for Epson IMU Linux C Driver using UART or SPI interface](#readme-for-epson-imu-linux-c-driver-using-uart-or-spi-interface)
- [Disclaimer:](#disclaimer)
- [Test machine:](#test-machine)
- [Requirements:](#requirements)
- [Important for UART Interface:](#important-for-uart-interface)
- [Important for RPi SPI Interface:](#important-for-rpi-spi-interface)
- [Compiling the software:](#compiling-the-software)
- [Important for configuring delays:](#important-for-configuring-delays)
- [Important for GPIO usage:](#important-for-gpio-usage)
- [How to run the program:](#how-to-run-the-program)
- [File listing:](#file-listing)
- [Change record:](#change-record)

<!---toc end-->

# Disclaimer:

THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, NONINFRINGEMENT,
SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE OR CLAIM, ARISING FROM OR
IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE SOFTWARE.

# Test machine:

- UART Interface:

  - Ubuntu 20.04 Mate running in Oracle VirtualBox on Core i7 Win10 PC
  - Raspberry Pi 3B+ (with Epson IMU USB evalboard)

- SPI Interface:

  - For using IMU SPI interface, this software is only tested on Raspberry Pi using wiringPi Library
  - RaspberryPi 3B+ with Ubuntu Mate-20.04-desktop-armhf
  - [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/)

# Requirements:

- For using IMU UART interface, this should work on any generic unix (POSIX) serial port with gcc or g++

# Important for UART Interface:

1. The application assumes that the Epson IMU is connected to serial tty (UART) either through USB evaluation board or directly to the UART port on an embedded system:
2. Edit the source files by specifying the proper serial port on host system:

```
const char *IMUSERIAL = "/dev/ttyxxx";
```

3. Specify the # of samples to capture in main_csvlogger.c or main_screen.c:

```
const unsigned int NUM_SAMPLES = xxxx;
```

# Important for RPi SPI Interface:

1. After the installation process, apply any updates.
   sudo apt-get update
   sudo apt-get upgrade

2. Use raspi-config utility to enable the SPI interface:
   sudo raspi-config

   Choose the advanced Options -> SPI -> enable SPI kernel module to be loaded by default "Yes"
   sudo reboot

3. Verify the SPI interface is enabled.
   lsmod | grep spi\_
   ls /dev | grep spi

4. Install the wiringPi library.
   This should already be installed if running a RPi-specific Linux distro, otherwise go to [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/)

The software is designed with the following pinmapping which can be modified by the user:

```
   Epson IMU                   Raspberry Pi
   ---------------------------------------------------
   EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22)
   EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24)
   EPSON_CS                    RPI_GPIO_P1_16 (GPIO23)
   SPI_SCK                     RPI_GPIO_P1_23 (GPIO11)
   SPI_MISO                    RPI_GPIO_P1_21 (GPIO9)
   SPI_MOSI                    RPI_GPIO_P1_19 (GPIO10)
```

# Compiling the software:

1. Run make clean  \<-- recommended before creating new builds
2. Run "make" specifying the **target**, **MODEL=**, **IF=**, and **PLATFORM=** parameters.
   - Supported **target** options are:

     - screen
     - csvlogger
     - regdump
     - all (if not specified)

   - Supported **MODEL=** parameters are:

     - G320PDG0
     - G330PDG0
     - G354PDH0
     - G364PDC0
     - G364PDCA
     - G365PDC1
     - G365PDF1
     - G366PDG0 (if not specified)
     - G370PDF1
     - G370PDS0
     - G370PDG0
     - G370PDT0
     - V340

   - Supported **IF=** options are:

     - UART (if not specified)
     - SPI

   - Supported **PLATFORM=** options are:

     - NONE (if not specified, means standard PC)
     - RPI

   - Example commands:

     - make screen MODEL=G370PDF1 IF=SPI PLATFORM=RPI
     - make csvlogger MODEL=G330PDG0
     - make regdump MODEL=G364PDC0 IF=SPI PLATFORM=RPI

The executable will be in the found in the same folder as the Makefile and source files.

NOTE: Modify the EpsonOptions struct to configure sensor configuration settings in main() function in main_xxxx.c

NOTE: Any references to SPI or GPIO interface refers to RaspberryPi only (For PLATFORM=NONE, they placeholders and currently do nothing).
The end user is required to connect the low level code to these GPIO functions to make use of them on an embedded platform since PCs do not have GPIO or SPI interfaces.

# Important for configuring delays:

NOTE: In the hcl_linux.c, there are functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.

On embedded Linux platforms, these may need to be redirected to HW specific delay routines if usleep() is not supported.

For example on RaspberryPi, the time delay functions for millisecond and microseconds in hcl_rpi.c are redirected to WiringPi library delay() and delayMicroseconds(), respectively.

# Important for GPIO usage:

When this driver connects to the IMU using the UART interface, the use of GPIO pins for connecting to the IMU RESET#, EXT, or DRDY is purely optional and only intended for embedded Linux platforms (non-PC based).

When this driver connects to the IMU using the SPI interface, the use of GPIO pins for connecting to the DRDY is **MANDATORY**.

When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization for better robustness.

This code does not implement GPIO functions. However, the code is structured to easily redirect to low-level hardware GPIO function calls for ease of implementation.
Review the *\_rpi* files as a template for implementation.

There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
  src/hcl_linux.c
  src/hcl_gpio.c
  src/hcl_gpio.h
```

Typically, an external library needs to be invoked to initialize the library & GPIO HW functions.

This requires the following changes to hcl_linux.c

- add #include to external library near the top of hcl_linux.c
- add #include hcl_gpio.h near the top of the hcl_linux.c
- add the initialization call inside the seInit() function in hcl_linux.c

For example on an Raspberry Pi with wiringPi, changes to hcl_linux.c:

```
  ...
  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library

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

Typically, the GPIO pins need to be assigned according to pin numbering specific to the HW platform.

This requires changes to hcl_gpio.h

For example on an Raspberry Pi with wiringPi, changes to hcl_gpio.h with the following pinmapping:

```
	Epson IMU                   Raspberry Pi
	---------------------------------------------------
	EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
	EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input

```

```
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

Typically, the external library will have GPIO pin control such as set_output, set_input, set, reset, read_pin_level, etc...

This requires changes to hcl_gpio.c

Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.

For example on an Raspberry Pi, changes to hcl_gpio.c:

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

# How to run the program:

1. Run the executable from console (may require root access to execute if regular user can not access TTY)

```
   sudo ./<executable filename>
```

2. The default *csvlogger* or *sreen* program creates CSV log of sensor data in a processed scaled log file for 1000 samples:

- Output date rate = 125 Hz

- Filter Tap = Moving Average TAP32

- Sensor Output:

  - NDFlags 16-bit Gyro X,Y,Z Accel X,Y,Z, ResetCounter Checksum, For G320PDG0, G330PDG0, G354PDH0, G364PDC0, G364PDCA, G365PDC1, G365PDF1, G366PDG0, G370PDF1, G370PDS0, G370PDG0, G370PDT0

  - NDFlags 16-bit Gyro X,Y,Z Accel X,Y,Z SampleCounter, For V340

**NOTE:** Output fields can be enabled or disabled in the *EpsonOptions struct* of the main_xxx.c file by setting the desired xxx_out to 1 (enabled) or 0 (disabled)

# File listing:

```
epson_imu_spi_ros_node.cpp    - ROS Driver C++ wrapper for SPI interface
epson_imu_uart_ros_node.cpp   - ROS Driver C++ wrapper for UART interface

hcl.h                       - Abstraction layer header for top-level hardware platform functions. This typically does not need modifying.
hcl_linux.c                 - Abstraction layer for top-level generic PC Linux
hcl_rpi.c                   - Abstraction layer for top-level RaspberryPi using wiringPi library
hcl_gpio.h                  - Header for GPIO abstraction layer. Modify GPIO pin assignments as needed for host platform
hcl_gpio.c                  - GPIO abstraction layer for connections to RESET, DRDY, CS# (dummy assignments for PC). Modify to redirect to GPIO library calls as needed for host platform
hcl_gpio_rpi.c              - GPIO abstraction layer for RaspberryPi using wiringPi library for connection to RESET, DRDY, CS#
hcl_spi.h                   - Header for SPI IF abstraction layer
hcl_spi_rpi.c               - SPI abstraction layer for RaspberryPi uses wiringPi library for SPI transactions
hcl_uart.h                  - Header for UART IF abstraction layer
hcl_uart.c                  - UART abstraction layer for standard Linux termios library for UART transactions
main_csvlogger.c            - Test application - Initialize IMU, and read sensor data to CSV log file
main_regdump.c              - Test application - Output register settings to console for debug purpose
main_screen.c               - Test application - Initialize IMU, and read sensor data to console
main_helper.c               - Helper functions for test application
main_helper.h               - Header for helper functions
Makefile                    - For make utility to compile test applications
README_src.md               - This file
sensor_epsonCommon.c        - Common functions for Epson IMU
sensor_epsonCommon.h        - Header for common functions for Epson IMU
sensor_epsonG320.c          - Model specific functions for Epson M-G320
sensor_epsonG320PDG0.h      - Model specific header for Epson M-G320PDG0
sensor_epsonG330_G366.c     - Model specific functions for Epson M-G330PDG0, M-G366PDG0
sensor_epsonG330PDG0.h      - Model specific header for Epson M-G330PDG0
sensor_epsonG354.c          - Model specific functions for Epson M-G354
sensor_epsonG354PDH0.h      - Model specific header for Epson M-G354PDH0
sensor_epsonG364.c          - Model specific functions for Epson M-G364PDC0, M-G364PDCA
sensor_epsonG364PDC0.h      - Model specific header for Epson M-G364PDC0
sensor_epsonG364PDCA.h      - Model specific header for Epson M-G364PDCA
sensor_epsonG365.c          - Model specific functions for Epson M-G365PDC1, M-G365PDF1
sensor_epsonG365PDC1.h      - Model specific header for Epson M-G365PDC1
sensor_epsonG365PDF1.h      - Model specific header for Epson M-G365PDF1
sensor_epsonG366PDG0.h      - Model specific header for Epson M-G366PDG0
sensor_epsonG370.c          - Model specific functions for Epson M-G370PDF1, M-G370PDS0, M-G370PDG0, M-G370PDT0,
sensor_epsonG370PDF1.h      - Model specific header for Epson M-G370PDF1
sensor_epsonG370PDS0.h      - Model specific header for Epson M-G370PDS0
sensor_epsonG370PDG0.h      - Model specific header for Epson M-G370PDG0
sensor_epsonG370PDT0.h      - Model specific header for Epson M-G370PDT0
sensor_epsonV340.c          - Model specific functions for Epson V340
sensor_epsonV340.h          - Model specific header for Epson V340PDD0
sensor_epsonSpi.c           - Register read/write and burst read functions with low-level SPI calls
sensor_epsonUart.c          - Register read/write and burst read functions with low-level UART calls
```

# Change record:

```
2023-11-08  v1.0.0    - Merge UART Driver v1.9 and SPI Driver v1.7, and minor updates
```
