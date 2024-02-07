// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorID,
                  kFrontLeftTurningMotorID,
                  kFrontLeftAbsoluteTurningEncoderID,
                  kFrontleftAngleOffset,
                  45.0,
                  "Front Left"},

      m_rearLeft{kRearLeftDriveMotorID,
                 kRearLeftTurningMotorID,
                 kRearLeftAbsoluteTurningEncoderID,
                 kRearleftAngleOffset,
                 -45.0,
                 "Rear Left "},

      m_frontRight{kFrontRightDriveMotorID,
                   kFrontRightTurningMotorID,
                   kFrontRightAbsoluteTurningEncoderID,
                   kFrontRightAngleOffset,
                   -45.0,
                   "Front Right "},

      m_rearRight{kRearRightDriveMotorID,
                  kRearRightTurningMotorID,
                  kRearRightAbsoluteTurningEncoderID,
                  kRearRightAngleOffset,
                  45.0,
                  "Rear Right "},
      m_odometry{kDriveKinematics,
                 GetPigeonRotation2D(),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(GetPigeonRotation2D(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot, bool fieldRelative,
                           units::second_t period) {
  auto states =
      kDriveKinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
          fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                              xSpeed, ySpeed, rot, GetPigeonRotation2D())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
          period));

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}


units::degree_t DriveSubsystem::GetHeading() const {
  return units::degree_t(m_pigeon->GetYaw());
}

void DriveSubsystem::ZeroHeading() {
  m_pigeon->SetYaw(0.0);
}

double DriveSubsystem::GetTurnRate() {
    double xyz_dps[3];
  m_pigeon->GetRawGyro(xyz_dps);  // returns degrees per second
  return xyz_dps[2];
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

frc::Rotation2d DriveSubsystem::GetPigeonRotation2D() {
  // The Pigeon doesn't return a Rotation2D object
  // So we had to make a method that got the degree and returned as a Rotation2D
  return frc::Rotation2d(GetHeading());
}
