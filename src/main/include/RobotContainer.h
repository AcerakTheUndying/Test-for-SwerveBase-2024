// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/deprecated.h>
WPI_IGNORE_DEPRECATED

//#include <frc/XboxController.h>
#include <frc/Joystick.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"

// Subsystems
#include "subsystems/DriveSubsystem.h"
//Commands
#include "commands/DefaultDrive.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  double ApplyDeadband(double joystickValue, double deadband);
  double ApplyDeadbandSquaredInputs(double joystickValue, double deadband);



  frc2::CommandPtr GetAutonomousCommand();

 private:
  // The driver's controller
  //frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
 frc::Joystick m_stick{OIConstants::kDriverControllerPort};
  
  

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;

  void ConfigureButtonBindings();
};
WPI_UNIGNORE_DEPRECATED