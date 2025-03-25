// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/Timer.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/FourBarSubsystem.h"
#include "subsystems/IntakePitchSubsystem.h"


/**
 * An example drive command that uses an drive subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SimpleAuto
    : public frc2::CommandHelper<frc2::Command, SimpleAuto> {
 public:
  /**
   * Creates a new SimpleAuto.
   *
   * @param drive The pointer to the drive subsystem
   */
  explicit SimpleAuto(DriveSubsystem* drive, FourBarSubsystem* fourBar, IntakePitchSubsystem* pitch);

  void Initialize() override; // Initializes
  bool IsFinished() override; // Can sample states to determine if command needs to end
  void End(bool interrupted) override; // Runs once after command is finnished

 private:
  // Declare private subsystem pointers to refrence real subsystmes
  DriveSubsystem* m_drive;
  FourBarSubsystem* m_fourBar;
  IntakePitchSubsystem* m_pitch;
  frc::Timer m_autoTimer;

};
