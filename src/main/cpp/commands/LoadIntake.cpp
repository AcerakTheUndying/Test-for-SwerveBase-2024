// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LoadIntake.h"

LoadIntake::LoadIntake(IntakeSubsystem* intakeSubsystem)
  : m_intakeSS(intakeSubsystem)
 {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intakeSubsystem);
  this->SetName("ExampleCommand");
}

// Called when the command is initially scheduled.
void LoadIntake::Initialize() {
  m_intakeSS->IntakeInwards(0.3);
}

// Called repeatedly when this Command is scheduled to run
void LoadIntake::Execute() {
  if (m_intakeSS->ProximitySensorOnReverseLimitConnector()) {
    m_intakeSS->IntakeInwards(0.1);
  }
}

// Called once the command ends or is interrupted.
void LoadIntake::End(bool interrupted) {
  m_intakeSS->IntakeStop();
}

// Returns true when the command should end.
bool LoadIntake::IsFinished() {
  return m_intakeSS->ProximitySensorOnForwardLimitConnector();
}
