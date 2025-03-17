// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/FourBarSubsystem.h"

/**
 * An example drive command that uses an drive subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class StopFourBar
    : public frc2::CommandHelper<frc2::Command, StopFourBar> {
 public:
  /**
   * Creates a new StopFourBar.
   *
   * @param fourbar The pointer to the fourbar subsystem
   */
  explicit StopFourBar(FourBarSubsystem* fourBar);

  void Initialize() override; // Initializes
  void Execute() override; // Main loop that runs
  bool IsFinished() override; // Can sample states to determine if command needs to end
  void End(bool interrupted) override; // Runs once after command is finnished

 private:
  // Declare private subsystem pointers to refrence real subsystmes
  FourBarSubsystem* m_fourBar;
  frc::XboxController* m_driveController;

};
