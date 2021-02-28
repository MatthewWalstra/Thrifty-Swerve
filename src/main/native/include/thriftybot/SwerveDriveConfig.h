/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <AHRS.h>
#include <frc/TimedRobot.h>

#include <vector>
#include <memory>

#include "thriftybot/MotorControllerConfig.h"

namespace Thriftybot {

//forward declare swerve drive and wheel classes to avoid circular dependencies
class SwerveDrive;
class Wheel;

class SwerveDriveConfig {
 public:
  static const int WHEEL_COUNT = 4;
  
  SwerveDriveConfig(){}

  //creates wheels for SwerveDrive
  std::array<std::shared_ptr<Wheel>, WHEEL_COUNT> getWheels();

  
  /**
   * NavX gyro connected to MXP SPI port, used for field-oriented driving. If null, field-oriented
   * driving is disabled.
   */
  std::shared_ptr<AHRS> gyro;

  /** Initialize with four initialized wheels, in order from wheel 0 to wheel 3. */
  std::array<std::shared_ptr<Wheel>, WHEEL_COUNT> wheels;

  /** Wheel base length from front to rear of robot. */
  double length = 1.0;

  /** Wheel base width from left to right of robot. */
  double width = 1.0;

  /**
   * Robot period is the {@code TimedRobot} period in seconds, defaults to {@code
   * TimedRobot.kDefaultPeriod}.
   */
  double robotPeriod = frc::TimedRobot::kDefaultPeriod.value();

  /** Factor to correct gyro lag when simultaneously applying azimuth and drive. **/
  double gyroRateCoeff = 0.0;

  /** System Dependent: switch if wheels form an x rather than a circle when only applying yaw to rotate in place (right x stick) **/
  bool invertError = true;

  /** Sets number of azimuth ticks per rotation **/
  int azimuthTicks = 4096;

  /** Enables Smartdashboard debugging info **/
  bool enableSmartDashboardOutput = true;

  double driveSetpointMax = 5500.0;

  /** Configs for the azimuth and drive motor controllers **/
  MotorControllerConfig azimuthConfig{MotorControllerConfig::AzimuthMotorController::TALON_SRX, MotorControllerConfig::FeedbackSensor::CTRE_MAG_ENCODER};
  MotorControllerConfig driveConfig{MotorControllerConfig::DriveMotorController::SPARK_MAX, MotorControllerConfig::FeedbackSensor::INTEGRATED_SENSOR};

};


}