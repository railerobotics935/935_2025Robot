// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "commands/climber/StopClimber.h"

StopClimber::StopClimber(ClimberSubsystem* climber)
    : m_climber{climber}  {
  // Register that this command requires the subsystem.
  AddRequirements(m_climber);
}

void StopClimber::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "StopClimber Initialized\r\n";
#endif
}

void StopClimber::Execute() {
  // Main execute loop that runs during the command
  m_climber->SetMotorPower(0.0);
}

bool StopClimber::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void StopClimber::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
  m_climber->SetMotorPower(0.0);
#ifdef PRINTDEBUG
  std::cout << "StopClimber Ended\r\n";
#endif
}