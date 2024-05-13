// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// WDR: This command will be the default command on the drive subsystem.
//  This means this command will be controlling robot unless something else requires the
//  drivetrain

#include <commands/DefaultDrive.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <frc/DriverStation.h>

DefaultDrive::DefaultDrive(DriveSubsystem *subsystem, std::function<double()> fwd_back,
                           std::function<double()> left_right, std::function<double()> rotation)
    : m_DriveSubsystem{subsystem},
      m_fwd_back{fwd_back},
      m_left_right{left_right},
      m_rotation{rotation}
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(subsystem);
  this->SetName("DefaultDrive");
}

// Called when the command is initially scheduled.
void DefaultDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DefaultDrive::Execute()
{
int invert = 1;
if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    invert = -1;
}
  m_DriveSubsystem->Drive(units::meters_per_second_t(m_fwd_back()*invert),
                          units::meters_per_second_t(m_left_right()*invert),
                          units::radians_per_second_t(m_rotation()),
                          true);
}

// Called once the command ends or is interrupted.
void DefaultDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool DefaultDrive::IsFinished()
{
  return false;
}
