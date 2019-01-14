# ADIS16470 IMU Library for FIRST Robotics and the RoboRIO

## ***FRC TEAMS, PLEASE BE AWARE THAT THE 2019 KICKOFF RELEASE IMAGE BREAKS COMPATIBILITY WITH ADI GYROS and IMUs! A UTILITY THAT AUTOMATICALLY PERFORMS THE WORKAROUND DESCRIBED ON THE WPILIB SCREENSTEPS PAGE IS AVAILABLE [HERE](https://github.com/juchong/ADIS16448-RoboRIO-Driver/releases/download/2019.r1/RoboRIODriverUpdateUtility.exe).***

## ***As described on the [WPI Known Issues](https://wpilib.screenstepslive.com/s/currentCS/m/getting_started/l/1028964-known-issues) page, the issue can be resolved by logging in to your RoboRIO via serial or SSH terminal as admin (password blank, usually), and executing the command "updateNIDrivers" without quotes. Note that this command is case sensitive. Future updates will resolve this issue and this step is only required to be executed once after re-imaging your RoboRIO with the kickoff image. If you run into any issues, please submit an issue ticket and we will reach out as quickly as possible.***

## Introduction
This example library was written to give mentors, students, and engineers a starting point for using a very high-performance 6 Degree-of-Freedom (DoF), calibrated Inertial Measurement Unit (IMU). This sensor packages multiple gyroscopes and accelerometers in a tiny, robust package perfect for high-performance robotics (such as FRC). 

These software libraries provide the user (you) with:
- Raw sensor outputs - X-Y-Z Gyroscope and Accelerometer
- X-Y-Z gyroscope angles calculated using loop integration
- Offset compensation calculated at runtime

Tutorial videos, how-to guides, and additional resources can be found on the [ADI FIRST Robotics Wiki Page](https://wiki.analog.com/first/first_robotics_donation_resources).

## What programming languages are supported?

The IMU driver currently supports all three official FRC languages (C++, Java, and LabVIEW). Raw sensor rate outputs and accumulated sensor outputs are supported for all languages. 

## What do I need to get started?

To use the software, you need access to a RoboRIO and the ADIS16470 RoboRIO Breakout Board. This software is based on the FRC 2019 software distribution and relies on the WPILib libraries to interface with the IMU. Previous (pre-2019) versions of LabVIEW and WPILib libraries are **not** supported. 

Plug in the expansion board as shown below. **Be careful not to offset the connector!!** If installed correctly, the Power LED should turn on once the RoboRIO is powered on.

Your RoboRIO should be imaged to match the version of the NI Update Suite installed on your PC. For example, if you have the latest (of this writing) update suite installed (2019.b3), then you must have the **FRC_roboRIO_2019_v9** image and **roboRIO_6.0.0** firmware installed as well. This driver relies heavily on the FPGA image loaded in the RoboRIO and _**will not work**_ on older versions. The most current NI Update Suite can be found [here](https://forums.ni.com/t5/FIRST-Robotics-Competition/FRC-Update-Suite/ta-p/3737502).

![ADIS16470 Breakout Board Installed on a RoboRIO](https://raw.githubusercontent.com/juchong/ADIS16470-RoboRIO-Driver/master/Reference/RioSensorBoard.jpg)

## How do I use the IMU with my programming language?

Click on the language you're looking to use above. Each folder includes instructions specific to the language specified. If you're looking for more information on using the sensor, be sure to check out the [ADI FIRST Robotics Wiki Page](https://wiki.analog.com/first/first_robotics_donation_resources).
