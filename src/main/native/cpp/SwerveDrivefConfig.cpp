#include "thriftybot/SwerveDriveConfig.h"

#include "thriftybot/SwerveDrive.h"
#include "thriftybot/Wheel.h"

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

std::array<std::shared_ptr<Thriftybot::Wheel>, Thriftybot::SwerveDriveConfig::WHEEL_COUNT> Thriftybot::SwerveDriveConfig::getWheels()
{

    // create wheel array
    std::array<std::shared_ptr<Thriftybot::Wheel>, Thriftybot::SwerveDriveConfig::WHEEL_COUNT> wheels;

    for (int i = 0; i < Thriftybot::SwerveDriveConfig::WHEEL_COUNT; i++)
    {
        //configure azimuthTalon
        std::shared_ptr<MotorControllerWrapper> azimuth;
        
        switch (azimuthConfig.azimuthController)
        {
            case MotorControllerConfig::AzimuthMotorController::VICTOR_SPX:
                azimuth = std::make_shared<VictorSPXWrapper>(azimuthConfig, i);
                break;
            case MotorControllerConfig::AzimuthMotorController::TALON_SRX:
                azimuth = std::make_shared<TalonSRXWrapper>(azimuthConfig, i);
                break;
            case MotorControllerConfig::AzimuthMotorController::SPARK_MAX:
                azimuth = std::make_shared<SparkMaxWrapper>(azimuthConfig, i);
                break;
            default:
                frc::DriverStation::ReportError("Invalid Azimuth Controller, defaulting to TalonSRX");
                azimuth = std::make_shared<TalonSRXWrapper>(azimuthConfig, i);
                break;
        }
        
        //configure DriveSparkMax
        std::shared_ptr<MotorControllerWrapper> drive;
        
        switch (driveConfig.driveController)
        {
            case MotorControllerConfig::DriveMotorController::SPARK_MAX:
                drive = std::make_shared<SparkMaxWrapper>(driveConfig, i + 10);
                break;
            case MotorControllerConfig::DriveMotorController::TALON_FX:
                drive = std::make_shared<TalonFXWrapper>(driveConfig, i + 10);
                break;
            default:
                frc::DriverStation::ReportError("Invalid Drive Controller, defaulting to SparkMax");
                drive = std::make_shared<SparkMaxWrapper>(driveConfig, i + 10);
                break;
        }
        
        std::shared_ptr<Thriftybot::Wheel> wheel = std::make_shared<Thriftybot::Wheel>(azimuth, drive, driveSetpointMax, i, azimuthTicks, invertError);
        wheels[i] = wheel;
    }

    return wheels;
}