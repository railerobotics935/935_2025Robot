// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Constants.h"
#include "commands/intake/IntakeWithTriggers.h"

IntakeWithTriggers::IntakeWithTriggers(IntakeSubsystem* intake, frc::XboxController* operatorController)
    : m_intake{intake}, m_operatorController{operatorController} {
  // Register that this command requires the subsystem.
  AddRequirements(m_intake);
}

void IntakeWithTriggers::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "IntakeWithTriggers Initialized\r\n";
#endif
}

void IntakeWithTriggers::Execute() {

  // Main execute loop that runs during the command
  const auto intakeSpeed = frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kIntakeTriggerIndex), 0.05);
  const auto outtakeSpeed = frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kOuttakeTriggerIndex), 0.05);
  
  if(!m_intake->CoralInIntake()) {
    if(intakeSpeed > 0.0) {
      m_intake->SetIntakeMotorPower(-intakeSpeed / 2);
    }
    else if(outtakeSpeed > 0.0) {
      m_intake->SetIntakeMotorPower(outtakeSpeed / 2);
    }
    else {
      m_intake->SetIntakeMotorPower(0.0);
    }
  }
  else if (outtakeSpeed > 0.0) {
  m_intake->SetIntakeMotorPower(outtakeSpeed / 2);
  }
  else {
    m_intake->SetIntakeMotorPower(0.0);
  }
  
  //m_intake->SetIntakeMotorPower(0.1);

}

bool IntakeWithTriggers::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void IntakeWithTriggers::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "IntakeWithTriggers Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
}