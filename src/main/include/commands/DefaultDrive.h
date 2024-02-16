// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//WDR: This command will be the default command on the drive subsystem.  
// This means this command will be controlling robot unless another command requires the 
// drivetrain through an addrequirements.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */


class DefaultDrive
    : public frc2::CommandHelper<frc2::Command, DefaultDrive> {
 public:
  DefaultDrive(DriveSubsystem* subsystem, std::function<double()> fwd_back,
                std::function<double()>left_right, std::function<double()>rotation);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;


  private:
    double m_fixedRobotHeading;
  DriveSubsystem * m_DriveSubsystem;
  std::function<double()> m_fwd_back;
  std::function<double()> m_left_right;
  std::function<double()> m_rotation;
};
