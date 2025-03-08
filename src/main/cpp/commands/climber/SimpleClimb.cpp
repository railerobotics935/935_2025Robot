// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "commands/climber/SimpleClimb.h"

SimpleClimb::SimpleClimb(ClimberSubsystem* climber)
    : m_climber{climber} {
  // Register that this command requires the subsystem.
  AddRequirements(m_climber);
}

void SimpleClimb::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "SimpleClimb Initialized\r\n";
#endif
}

void SimpleClimb::Execute() {
  // Main execute loop that runs during the command
  m_climber->SetMotorPower(1.0);
}

bool SimpleClimb::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void SimpleClimb::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "SimpleClimb Ended\r\n";
#endif
    m_climber->SetMotorPower(0.0);
}