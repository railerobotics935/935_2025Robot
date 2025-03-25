// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "Constants.h"
#include "commands/autocommands/SimpleAuto.h"

SimpleAuto::SimpleAuto(DriveSubsystem* drive, FourBarSubsystem* fourBar, IntakePitchSubsystem* pitch)
    : m_drive{drive}, m_fourBar{fourBar}, m_pitch{pitch}{
  // Register that this command requires the subsystem.
  AddRequirements(m_drive), AddRequirements(m_fourBar), AddRequirements(m_pitch);
}

void SimpleAuto::Initialize() {
  // Run once when command is scheduled
#ifdef PRINTDEBUG
  std::cout << "SimpleAuto Initialized\r\n";
#endif
m_fourBar->SetFourBarHeight(0.265);
m_drive->Drive((units::meters_per_second_t)0.1, (units::meters_per_second_t)0.0, (units::radians_per_second_t)0.0, true);
//m_intakePitch->SetPitchPosition(0.675);
m_autoTimer.Start();
}


bool SimpleAuto::IsFinished() {
  // You can make a custom state to end the command and then return true
  if (
    m_autoTimer.Get() >= (units::second_t)3.0 
  ) {
         return true;
  }
  return false;
}

void SimpleAuto::End(bool interrupted) {
  // Runs once when the command is removed from the command scheduler
#ifdef PRINTDEBUG
  std::cout << "SimpleAuto Ended\r\n";
#endif
  m_drive->Drive((units::meters_per_second_t)0.0, (units::meters_per_second_t)0.0, (units::radians_per_second_t)0.0, true);
  m_fourBar->SetFourBarHeight(0.0);
}