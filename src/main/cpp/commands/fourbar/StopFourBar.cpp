// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "commands/fourbar/StopFourBar.h"

StopFourBar::StopFourBar(FourBarSubsystem* fourBar)
    : m_fourBar{fourBar}    {
  // Register that this command requires the subsystem.
  AddRequirements(m_fourBar);
}

void StopFourBar::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "StopFourBar Initialized\r\n";
#endif
}

void StopFourBar::Execute() {
  // Main execute loop that runs during the command
  //m_fourBar->SetFourBarPower(0.0);
}

bool StopFourBar::IsFinished() {
  // You can make a custom state to end the command and then return true
  return false;
}

void StopFourBar::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "StopFourBar Ended\r\n";
#endif
   // m_fourBar->SetFourBarPower(0.0);
}