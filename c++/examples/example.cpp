/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cmath>

#include <Drive/DifferentialDrive.h>
#include <IterativeRobot.h>
#include <Joystick.h>
#include <Spark.h>
#include <ADIS16470_IMU.h>

/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a
 * robot drive straight. This program uses a joystick to drive forwards and
 * backwards while the gyro is used for direction keeping. This code assumes
 * that your yaw axis is the X axis.
 */
class Robot : public frc::IterativeRobot {
public:
	
	ADIS16470_IMU imu*;
	
	void RobotInit() override {
		imu.Calibrate();
                imu.SetYawAxis(kX);
	}

	/**
	 * The motor speed is set from the joystick while the DifferentialDrive
	 * turning value is assigned from the error between the setpoint and the
	 * gyro angle.
	 */
	void TeleopPeriodic() override {
		double turningValue = (kAngleSetpoint - imu.GetAngle()) * kP;
		// Invert the direction of the turn if we are going backwards
		turningValue = std::copysign(turningValue, m_joystick.GetY());
		m_robotDrive.ArcadeDrive(m_joystick.GetY(), turningValue);
	}

private:
	static constexpr double kAngleSetpoint = 0.0;
        static constexpr double kP = 0.005;

	static constexpr int kLeftMotorPort = 0;
	static constexpr int kRightMotorPort = 1;
	static constexpr int kJoystickPort = 0;

	frc::Spark m_left{kLeftMotorPort};
	frc::Spark m_right{kRightMotorPort};
	frc::DifferentialDrive m_robotDrive{m_left, m_right};
        frc::ADIS16470_IMU imu{};

	frc::Joystick m_joystick{kJoystickPort};
};

START_ROBOT_CLASS(Robot)