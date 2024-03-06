// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <numbers>
#include "TargetRobot.h"
#include <frc/TimedRobot.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// Global Constants

static constexpr char const *RIO_CANBUS_NAME{"rio"};
static constexpr char const *CARNIVORE_CANBUS_NAME{"Canivore1"};

constexpr int kTimeoutMs = 10;
constexpr int kPIDLoopIdx = 0;

namespace DriveConstants {
constexpr int kPigeonID = 5;

// Drive Motor ID's end in 2
constexpr int kFrontLeftDriveMotorID = 22;
constexpr int kRearLeftDriveMotorID = 42;
constexpr int kFrontRightDriveMotorID = 12;
constexpr int kRearRightDriveMotorID = 32;

// Turning Motor ID's end in 1
constexpr int kFrontLeftTurningMotorID = 21;
constexpr int kRearLeftTurningMotorID = 41;
constexpr int kFrontRightTurningMotorID = 11;
constexpr int kRearRightTurningMotorID = 31;

// CANCoder ID's end in 0
constexpr int kFrontLeftAbsoluteTurningEncoderID = 20;
constexpr int kRearLeftAbsoluteTurningEncoderID = 40;
constexpr int kFrontRightAbsoluteTurningEncoderID = 10;
constexpr int kRearRightAbsoluteTurningEncoderID = 30;
constexpr double kTurning_kP = 0.22;
constexpr double kDrive_kP = 0.1;




// If you call DriveSubsystem::Drive with a different period make sure to update
// this.
inline constexpr units::second_t kDrivePeriod = frc::TimedRobot::kDefaultPeriod;

// These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
// These characterization values MUST be determined either experimentally or
// theoretically for *your* robot's drive. The SysId tool provides a convenient
// method for obtaining these values for your robot.
inline constexpr auto ks = 1_V;
inline constexpr auto kv = 0.8 * 1_V * 1_s / 1_m;
inline constexpr auto ka = 0.15 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
inline constexpr double kPFrontLeftVel = 0.5;
inline constexpr double kPRearLeftVel = 0.5;
inline constexpr double kPFrontRightVel = 0.5;
inline constexpr double kPRearRightVel = 0.5;

constexpr double kFrontleftAngleOffset =
    -124.717;  // 104.854;    //-178.505859;    //-295.4, Generic offset is
               // -43.505859
constexpr double kFrontRightAngleOffset =
    -81.65;  //-131.2, Generic offset is 58.535
constexpr double kRearleftAngleOffset =
    145.986;  //-62.2, Generic offset is 10.986
constexpr double kRearRightAngleOffset =
    -34.98047;  //-224.4, Generic offset 145.01953
constexpr double kSteeringRatio = (60.0 / 10.0) * (50.0 / 14.0);

constexpr auto kMaxDrivingRotation = units::radians_per_second_t(2.5);

}  // namespace DriveConstants

namespace IntakeConstants {
// Intake Motor
constexpr int kIntakeMotorCANID = 50; 

}  // namespace IntakeConstants 

namespace ClimbConstants {
// Climb Motors
constexpr int kLeftClimberMotorCANID = 61;
constexpr int kRightClimberMotorCANID = 62;
}  // namespace ClimbConstants

namespace ModuleConstants {
constexpr double kDriveGearRatio =
    (50.0 / 14.0) * (17 / 27.0) * (45.0 / 15.0);  // L2 Drive gear ratio

inline constexpr int kEncoderCPR = 2048;
inline constexpr double kWheelDiameterMeters = 0.10033;

// 2024 Robot
#ifdef ROBOT2024
constexpr double kTurnGearRatio =
    (50.0 / 14.0) * (60.0 / 10.0);  // L2 Turn gear ratio
#endif

// spo_robot
#ifdef SPOBOT
constexpr double kTurnGearRatio =
    (32.0 / 15.0) * (60.0 / 10.0);  // L2 Turn gear ratio
#endif

constexpr double kDriveEncoderMetresPerPulse =
    // using encoder inside Falcon so need to divide circumference by gear ratio
    // and by CPR
    (kWheelDiameterMeters * std::numbers::pi) /
    static_cast<double>(kEncoderCPR) / kDriveGearRatio;

constexpr double kTurningEncoderRadiansPerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    (std::numbers::pi * 2) / static_cast<double>(kEncoderCPR) / kTurnGearRatio;

inline constexpr double kPModuleTurningController = 1;
inline constexpr double kPModuleDriveController = 1;
}  // namespace ModuleConstants

namespace AutoConstants {
inline constexpr auto kMaxSpeed = 3_mps;
inline constexpr auto kMaxAcceleration = 3_mps_sq;
inline constexpr auto kMaxAngularSpeed = 3.142_rad_per_s;
inline constexpr auto kMaxAngularAcceleration = 3.142_rad_per_s_sq;

inline constexpr double kPXController = 0.5;
inline constexpr double kPYController = 0.5;
inline constexpr double kPThetaController = 0.5;

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;
}  // namespace AutoConstants

namespace TeleopConstants {
inline constexpr auto kMaxSpeedInTeleop = 5_mps;
inline constexpr auto kMaxAccelerationInTeleop = 5_mps_sq;
inline constexpr auto kMaxAngularSpeedInTel3op = 3.142_rad_per_s;
inline constexpr auto kMaxAngularAccelerationInTeleop = 3.142_rad_per_s_sq;
}  // namespace TeleopConstants

namespace OIConstants {
inline constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants

namespace ShooterConstants {
const int shooterMotorID = 3;
const int feedMotorID = 4;
const int elevatorMotorID = 5;
}  // namespace ShooterConstants