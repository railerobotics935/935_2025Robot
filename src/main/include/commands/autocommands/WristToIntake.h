#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakePitchSubsystem.h"

class SetWristToIntake
  : public frc2::CommandHelper<frc2::Command, SetWristToIntake> {
public:
  /**
   * Creates a new command for setting wrist to L1.
   *
   * @param intakepitch The pointer to the pitch subsystem
   */
  explicit SetWristToIntake(IntakePitchSubsystem *intakepitch);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  IntakePitchSubsystem* m_intakePitch;
};
