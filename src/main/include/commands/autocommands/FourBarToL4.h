
#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/FourbarSubsystem.h"

class FourBarToL4
  : public frc2::CommandHelper<frc2::Command, FourBarToL4> {
public:
  /**
   * Creates a new FourBarSetPoint.
   *
   * @param fourBar The pointer to the intake subsystem
   */
  explicit FourBarToL4(FourBarSubsystem* fourBar);

  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  FourBarSubsystem* m_fourBar;

  // Encoder motor controllers
};
