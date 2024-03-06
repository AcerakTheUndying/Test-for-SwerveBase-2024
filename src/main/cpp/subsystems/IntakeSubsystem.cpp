// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IntakeSubsystem.h"
#include <ctre/phoenix6/StatusSignal.hpp>
#include <frc2/command/FunctionalCommand.h>


IntakeSubsystem::IntakeSubsystem() = default;

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic() {
 // if (ProximitySensorOnForwardLimitConnector()){
 //   fmt::println("Forward Limit {}",m_intakeMotor.GetForwardLimit().ToString());
 // }
 // if (ProximitySensorOnReverseLimitConnector()){
 //   fmt::println("Reverse Limit {}",m_intakeMotor.GetReverseLimit().ToString());
 // }
}

void IntakeSubsystem::RotateToForward(){
  // Code here to mnotion magic to the forward position
};

void IntakeSubsystem::RotateToBackwards(){
    // Code here to mnotion magic to the backwards position
};

void IntakeSubsystem::RotateToLeft(){
    // Code here to mnotion magic to the left position
};

void IntakeSubsystem::RotateToRight(){
    // Code here to mnotion magic to the right position
};

void IntakeSubsystem::IntakeInwards(double IntakePower){
  m_intakeMotor.Set(IntakePower);
}

void IntakeSubsystem::IntakeStop(){
  m_intakeMotor.Set(0.0);
  fmt::println("Motor Rotor Position {}",m_intakeMotor.GetRotorPosition().GetValueAsDouble());
  
}

void IntakeSubsystem::IntakeForwardUntilRollersClear(){
  //Code here to move the motor forward in position mode so that the root can move over a note without touching it.
}

bool IntakeSubsystem::ProximitySensorOnForwardLimitConnector(){
  //ctre::phoenix6::StatusSignal LimitObject= m_intakeMotor.GetForwardLimit();
  //if (LimitObject.GetValue()==ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround);
  return (m_intakeMotor.GetForwardLimit().GetValue()==ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround);
}

bool IntakeSubsystem::ProximitySensorOnReverseLimitConnector(){
  //ctre::phoenix6::StatusSignal LimitObject= m_intakeMotor.GetForwardLimit();
  //if (LimitObject.GetValue()==ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround);
  return (m_intakeMotor.GetReverseLimit().GetValue()==ctre::phoenix6::signals::ReverseLimitValue::ClosedToGround);
}
frc2::CommandPtr IntakeSubsystem::ExampleFullCommand(){
  return frc2::FunctionalCommand(
  // Reset encoders on command start //This is Command Init
  [this] { //m_drive.ResetEncoders(); 
          IntakeInwards(0.1);
  },
  // Start driving forward at the start of the command  //This is the Command PeriodicFunction
  [this] { //m_drive.ArcadeDrive(ac::kAutoDriveSpeed, 0); 
  },
  // Stop driving at the end of the command  //This is the Command End Function
  [this] (bool interrupted) { 
    //m_drive.ArcadeDrive(0, 0); 
        IntakeStop();
  },
  // End the command when the robot's driven distance exceeds the desired value  //This is the Command IsFinished
  [this] { 
    //return m_drive.GetAverageEncoderDistance() >= kAutoDriveDistanceInches; 
    return ProximitySensorOnForwardLimitConnector();
    //return false;
    }
  // Requires the subsystem

).ToPtr();
}

frc2::CommandPtr IntakeSubsystem::IntakeFromFront(){
    return this->RunOnce(
        [this] {RotateToForward(); 
        // Run intake until loaded or cancelled
                }
        );
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
