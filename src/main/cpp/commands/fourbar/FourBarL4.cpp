// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "commands/fourbar/FourBarL4.h"

FourBarL4::FourBarL4(FourBarSubsystem* fourBar)
    : m_fourBar{fourBar} {
  // Register that this command requires the subsystem.
  AddRequirements(m_fourBar);
}

void FourBarL4::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "FourBarL4 Initialized\r\n";
#endif
  m_fourBar->SetFourBarHeight(0.0);
}

void FourBarL4::Execute() {
  // Main execute loop that runs during the command
}

bool FourBarL4::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void FourBarL4::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "FourBarL4 Ended\r\n";
#endif
}