// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <utility>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>


using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
//  m_drive.SetDefaultCommand(DefaultDrive(
//      &m_drive,
//      [this] {
//        /*frc::SmartDashboard::PutNumber("Joystick X",
//                                       -m_driverController.GetY());*/
//        return /*m_xspeedLimiter.Calculate(*/ ApplyDeadband(-m_stick.GetY(),
//                                                            0.1) *
//               ((1 - m_stick.GetThrottle()) / 2) * 5.0 /*)*/;
//      },  // WDR:Need to implement constants for deadband and maxspeed
//      [this] {
//        return /*m_yspeedLimiter.Calculate(*/ ApplyDeadband(-m_stick.GetX(),
//                                                            0.1) *
//               ((1 - m_stick.GetThrottle()) / 2) * 5.0 /*)*/;
//      },  // WDR:Need to implement constants for deadband and maxspeed
//      [this] {
//        return /*m_rotLimiter.Calculate(*/ ApplyDeadbandSquaredInputs(
//                   -m_stick.GetZ(), 0.2) *
//               ((1 - m_stick.GetThrottle()) / 2) *
//               kMaxDrivingRotation.value() /*)*/;
//      }  // WDR:Need to implement constants for deadband and max rotation
//  ));

  m_drive.SetDefaultCommand(DefaultDrive(
      &m_drive,
      [this] { return (ApplyDeadbandSquaredInputs(-m_stick.GetY(),0.1)*((1 - m_stick.GetThrottle()) / 2) * 5.0);},
      [this] { return (ApplyDeadbandSquaredInputs(-m_stick.GetX(),0.1)*((1 - m_stick.GetThrottle()) / 2) * 5.0);},
      [this] { return (ApplyDeadband(-m_stick.GetZ(),0.1));}
  ));




}



void RobotContainer::ConfigureButtonBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::CommandPtr swerveControllerCommand =
      frc2::SwerveControllerCommand<4>(
          exampleTrajectory, [this]() { return m_drive.GetPose(); },

          m_drive.kDriveKinematics,

          frc::PIDController{AutoConstants::kPXController, 0, 0},
          frc::PIDController{AutoConstants::kPYController, 0, 0},
          thetaController,

          [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

          {&m_drive})
          .ToPtr();

  // Reset odometry to the initial pose of the trajectory, run path following
  // command, then stop at the end.
  return frc2::cmd::Sequence(
      frc2::InstantCommand(
          [this, &exampleTrajectory]() {
            m_drive.ResetOdometry(exampleTrajectory.InitialPose());
          },
          {})
          .ToPtr(),
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this] { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false); }, {})
          .ToPtr());
}

double RobotContainer::ApplyDeadband(double joystickValue, double deadband) {
  double result = 0.0;
  if (joystickValue < -deadband)
    result = (joystickValue + deadband) / (1.0 - deadband);

  if (joystickValue > deadband)
    result = (joystickValue - deadband) / (1.0 - deadband);
  return result;
}

double RobotContainer::ApplyDeadbandSquaredInputs(double joystickValue,
                                                  double deadband) {
  double result = 0.0;
  if (joystickValue < -deadband)
    result = (joystickValue + deadband) / (1.0 - deadband) *
             fabs(joystickValue + deadband) / (1.0 - deadband);

  if (joystickValue > deadband)
    result = (joystickValue - deadband) / (1.0 - deadband) *
             fabs(joystickValue - deadband) / (1.0 - deadband);
return result;             
}