// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();
   
  void RotateToForward();
  void RotateToBackwards();
  void RotateToLeft();
  void RotateToRight();

  void IntakeInwards(double IntakePower);
  void IntakeStop();

  frc2::CommandPtr IntakeFromFront();
  frc2::CommandPtr IntakeFromBack();
  frc2::CommandPtr IntakeFromLeft();
  frc2::CommandPtr IntakeFromRight();
  frc2::CommandPtr HandoverToShooter();

  bool ProximitySensorOnForwardLimitConnector();
  bool ProximitySensorOnReverseLimitConnector();

  frc2::CommandPtr ExampleFullCommand();  //Example to show how a full command can be created inline

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:


  void IntakeForwardUntilRollersClear();
  void EjectNoteToShooter();

 ctre::phoenix6::hardware::TalonFX m_intakeMotor{IntakeConstants::kIntakeMotorCANID,RIO_CANBUS_NAME};

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
