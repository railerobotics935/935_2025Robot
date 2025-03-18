// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "commands/fourbar/RaiseFourBar.h"

RaiseFourBar::RaiseFourBar(FourBarSubsystem* fourBar)
    : m_fourBar{fourBar}    {
  // Register that this command requires the subsystem.
  AddRequirements(m_fourBar);
}

void RaiseFourBar::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "RaiseFourBar Initialized\r\n";
#endif
}

void RaiseFourBar::Execute() {
  // Main execute loop that runs during the command
  m_fourBar->SetFourBarPower(0.3);
}

bool RaiseFourBar::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void RaiseFourBar::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "RaiseFourBar Ended\r\n";
#endif
    m_fourBar->SetFourBarPower(0.0);
}