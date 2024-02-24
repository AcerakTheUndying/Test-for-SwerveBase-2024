// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() = default;

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {}

void RotateToForward(){};
void RotateToBackwards(){};
void RotateToLeft(){};
void RotateToRight(){};

frc2::CommandPtr IntakeSubsystem::IntakeFromFront(){
    return this->RunOnce(
        [this] {RotateToForward(); });
  };

  frc2::CommandPtr IntakeSubsystem::IntakeFromBack(){
    return this->RunOnce(
        [this] { RotateToBackwards();
                //Intake until loaded
                }
    );
  };

  frc2::CommandPtr IntakeSubsystem::IntakeFromLeft(){
        return this->RunOnce(
        [this] { RotateToBackwards();
                //Intake until loaded
                }
    );
  };
  
  frc2::CommandPtr IntakeSubsystem::IntakeFromRight(){
        return this->RunOnce(
        [this] { RotateToBackwards();
                //Intake until loaded
                }
    );
  };

  frc2::CommandPtr IntakeSubsystem::HandoverToShooter(){
        return this->RunOnce(
        [this] { RotateToForward();
                // check Shooter ready to receive
                // Run intake until handover complete
                }
    );
  };
