/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "thriftybot/Wheel.h"
#include <frc/Timer.h>
#include <iostream>

//using namespace Thriftybot;

Thriftybot::Wheel::Wheel(std::shared_ptr<MotorControllerWrapper> azimuth, std::shared_ptr<MotorControllerWrapper> drive, double driveSetpointMax, int id, int ticks, bool invertError) 
{
    azimuthController = azimuth;
    driveController = drive;
    this->driveSetpointMax = driveSetpointMax;
    this->id = id;
    TICKS = ticks;
    this->invertError = invertError;
    
    setDriveMode(MotorControllerWrapper::DriveMode::TELEOP);
}

void Thriftybot::Wheel::set(double azimuth, double drive, bool output_smartdashboard)
{
    if (Util::epsilonEquals(drive, 0.0, 0.05))
    {
        driveController->set(0.0);
        return;
    }

    azimuth *= TICKS * (invertError ? -1.0 : 1.0); //flip azimuth: hardware configuration dependent

    double azimuthPosition = azimuthController->getPosition();
    azimuthError = std::fmod(azimuth - azimuthPosition, TICKS);

    //wrap ticks, so that it's between +-2048
    azimuthError -= azimuthError > TICKS / 2.0? TICKS : 0.0;

    //minimize azimuth rotation, reversing drive if necessary
    
    inverted = std::fabs(azimuthError) > .25 * TICKS;
    
    std::string flipped = "";

    if (inverted)
    {
        azimuthError -= std::copysign(0.5 * TICKS, azimuthError);
        drive = -drive;

        flipped = ": flipped ";
    }
    
    if (output_smartdashboard)
    {
        frc::SmartDashboard::PutBoolean("Flipped: Azimuth" + Util::sstr(id), inverted);
        frc::SmartDashboard::PutNumber("Pre error" + Util::sstr(id), std::fabs(azimuthError));
        frc::SmartDashboard::PutNumber("Thriftybot/Swerve/Timestamp_" + Util::sstr(id), frc::Timer::GetFPGATimestamp());
        frc::SmartDashboard::PutNumber("Thriftybot/Swerve/Drive_" + Util::sstr(id) + " setpoint", drive);
        frc::SmartDashboard::PutNumber("Thriftybot/Swerve/Azimuth_" + Util::sstr(id) + " setpoint", azimuthPosition + azimuthError);
        frc::SmartDashboard::PutNumber("Thriftybot/Swerve/Azimuth_" + Util::sstr(id) + " position", azimuthPosition);
        frc::SmartDashboard::PutNumber("Thriftybot/Swerve/Azimuth_" + Util::sstr(id) + " error", azimuthError);
        frc::SmartDashboard::PutNumber("Thriftybot/Swerve/Azimuth_" + Util::sstr(id) + " zero", azimuth_zero);
        frc::SmartDashboard::PutNumber("Thriftybot/Swerve/Azimuth_" + Util::sstr(id) + " abs position", getAzimuthAbsolutePosition());
    }

    stream << std::fixed << std::setprecision(4) <<frc::Timer::GetFPGATimestamp() << ","<< id << ","<< azimuthPosition << ","<< azimuthError << ","<< azimuth << ","<< flipped << ",";    

    azimuthController->set(azimuthPosition + azimuthError);
    driveController->set(drive);
}

void Thriftybot::Wheel::setAzimuthPosition(int position)
{
    azimuthController->set(position);
}

void Thriftybot::Wheel::disableAzimuth()
{
    azimuthController->setNeutralOutput();
}

void Thriftybot::Wheel::setDriveMode(MotorControllerWrapper::DriveMode driveMode)
{
    azimuthController->setDriveMode(driveMode);
    driveController->setDriveMode(driveMode);
}

void Thriftybot::Wheel::stop()
{
    azimuthController->set(azimuthController->getPosition());
    driveController->set(0.0);
}

void Thriftybot::Wheel::setAzimuthZero(int zero)
{
    azimuth_zero = zero;
    int azimuthSetpoint = getAzimuthAbsolutePosition() - zero;
    azimuthController->setSensorPosition(azimuthSetpoint);
}

int Thriftybot::Wheel::getAzimuthAbsolutePosition()
{
    return azimuthController->getAbsPosition();
}

std::string Thriftybot::Wheel::getString()
{
    std::string output = stream.str(); 
    stream = std::stringstream();
    return output;
}
