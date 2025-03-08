// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "commands/climber/SimpleDescend.h"

SimpleDescend::SimpleDescend(ClimberSubsystem* climber)
    : m_climber{climber}  {
  // Register that this command requires the subsystem.
  AddRequirements(m_climber);
}

void SimpleDescend::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "SimpleDescend Initialized\r\n";
#endif
}

void SimpleDescend::Execute() {
  // Main execute loop that runs during the command
  m_climber->SetMotorPower(-1.0);
}

bool SimpleDescend::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void SimpleDescend::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "SimpleDescend Ended\r\n";
#endif
    m_climber->SetMotorPower(0.0);
}