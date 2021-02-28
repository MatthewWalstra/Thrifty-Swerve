#pragma once

#include <AHRS.h>
#include <frc/TimedRobot.h>

#include <vector>
#include <memory>

namespace Thriftybot {

struct PIDFSlot
{
  double kP, kI, kD, kF, kIZone, kAllowableError, kMaxIAccum = 0.0; 
  
  PIDFSlot(bool azimuth)
  {
    //initialize default PIDF values based on whether it's an azimuth or drive slot
    if (azimuth)
    {
      //Azimuth: TODO - different for spark max and talons?
      kP = 2.0;
      kI = 0.0;
      kD = 30.0;
      kF = 0.0;
      kIZone = 0.0;
      kAllowableError = 0.0;
      kMaxIAccum = 0.0;
    } else
    {
      //Drive: TODO - get better defaults
      kP = 2.0;
      kI = 0.0005;
      kD = 30.0;
      kF = 0.032;
      kIZone = 1000.0;
      kAllowableError = 0.0;
      kMaxIAccum = 150000.0;
    }
    
  }
    

};

struct MotorControllerConfig
{
  private:
    
  public:
    bool isAzimuth = true;
    /** Supported Feedback Sensors **/
    enum FeedbackSensor
    {
      CTRE_MAG_ENCODER,
      CAN_CODER,
      THRIFTY_CODER,
      INTEGRATED_SENSOR
    } feedbackSensor = CTRE_MAG_ENCODER;

    /** Supported Drive Motor Controllers **/
    enum class DriveMotorController
    {
      SPARK_MAX,
      TALON_FX
    } driveController = DriveMotorController::SPARK_MAX;

    /** Supported Azimuth Motor Controllers **/
    enum class AzimuthMotorController
    {
      VICTOR_SPX,
      TALON_SRX,
      SPARK_MAX
    } azimuthController = AzimuthMotorController::TALON_SRX;

    /** Motor Controller Neutral Modes **/
    enum NeutralMode
    {
      BRAKE,
      COAST
    } neutralMode = BRAKE;

    /** update ONLY for Spark Max azimuth controllers with a brushed motor **/
    enum MotorType
    {
      BRUSHED,
      BRUSHLESS
    } motorType = BRUSHLESS;

    /**
     * peakCurrentLimit -> Talon: peakCurrentLimit, SparkMax: secondaryCurrentLimit
     * continuousCurrentLimit -> Talon: continuousCurrentLimit, SparkMax: smartCurrentLimit
     * 
     * motionAcceleration -> Talon: motionAcceleration, SparkMax.PIDController: smartMotionAcceleration
     * motionCruiseVelocity -> Talon: motionCruiseVelocity, SparkMax.PIDController: smartMotionMaxVelocity
     * 
     * voltageCompensation -> Talon: voltageCompSaturation, SparkMax: voltageCompensation
    **/
    double peakCurrentLimit, continuousCurrentLimit, motionAcceleration, motionCruiseVelocity, voltageCompensation = 0.0;
    
    MotorControllerConfig(AzimuthMotorController azimuth, FeedbackSensor sensor): isAzimuth(true)
    {
      //update values for azimuth
      azimuthController = azimuth;
      feedbackSensor = sensor;
      neutralMode = COAST;
      peakCurrentLimit = 30.0;
      continuousCurrentLimit = 15.0;
      motionAcceleration = 10000.0;
      motionCruiseVelocity = 800.0;
      voltageCompensation = 12.0;
    }

    MotorControllerConfig(DriveMotorController drive, FeedbackSensor sensor): isAzimuth(false)
    {
      //update values for drive
      driveController = drive;
      feedbackSensor = sensor;
      neutralMode = BRAKE;

      peakCurrentLimit = 80.0;
      continuousCurrentLimit = 60.0;
      motionAcceleration = 20000.0; // Should probably never be used, but stops the spark maxes from throwing an error.
      motionCruiseVelocity = 5500.0;
      voltageCompensation = 12.0;
    }
    
    //4 slots for each motor controller -- might need to be moved inside constructor
    PIDFSlot slot0 = PIDFSlot(isAzimuth);
    PIDFSlot slot1 = PIDFSlot(isAzimuth);
    PIDFSlot slot2 = PIDFSlot(isAzimuth);
    PIDFSlot slot3 = PIDFSlot(isAzimuth);
    

};

}