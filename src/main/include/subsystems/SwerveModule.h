// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>
#include <wpi/deprecated.h>
WPI_IGNORE_DEPRECATED

//2023
#include <ctre/Phoenix.h> // Needed for Falcons and Pigeon
//2024
#include "ctre/phoenix6/TalonFX.hpp"

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/trajectory/TrapezoidProfile.h>


#include "Constants.h"

using namespace OIConstants;
//#include <frc/WPILib.h>
class SwerveModule
{   
public:
    SwerveModule(int driveMotorCANBusID,
                 int turningMotorCANBusID,
                 int absoluteTurnEncoderID,
                 double offsetDegrees,
                 double m_brakeAngle,
                 std::string moduleName
                 
    );
    frc::SwerveModulePosition GetPosition();
    frc::SwerveModuleState GetState();
    void SetDesiredState(const frc::SwerveModuleState &state);
    void ResetEncoders();
    void SetParkMode();
    void ReleaseParkMode();

    units::radian_t GetSwerveTurningFalconInternalRadians();
    std::string m_name;


private:
    // We have to use meters here instead of radians due to the fact that
    // ProfiledPIDController's constraints only take in meters per second and
    // meters per second squared.

    /*static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
        units::radians_per_second_t(std::numbers::pi); // radians per second
    static constexpr units::unit_t<radians_per_second_squared_t>
        kModuleMaxAngularAcceleration =
            units::unit_t<radians_per_second_squared_t>(
                std::numbers::pi * 2.0); // radians per second squared
*/
    //2023 Phoenix5 way.
    WPI_TalonFX m_driveMotor;
    WPI_TalonFX m_turningMotor;
    CANCoder m_absoluteTurnEncoder; // This will be used to initialise the Falcons Built-in relative encoder 
    
    //2024 Migrating to Phoenix6
    

    double m_brakeAngle;
    
    // Variable to hold the number of times the swerve module has turned through a full revolution 
    // Increments when the desired heading changes from somewhere in the range -90 to -180 to +90 to +180
    // Decrements when the desired heading changes from somewhere in the range +90 to +180 to -90 to -180
    int m_encoderWindUpRevolutions=0;

    //Variable used to remember the last heading of the swerve module to calculate if a wrapping condition has occured
    double m_prevModuleAngleDegrees;
    
    //2024 Migrate to Phoenix 6
    // Need a velocity Control Object to put the drive into velocity control mode
    ctre::phoenix6::controls::VelocityVoltage m_voltageVelocity{0_tps, 0_tr_per_s_sq, true, 0_V, 0, false};
};

WPI_UNIGNORE_DEPRECATED