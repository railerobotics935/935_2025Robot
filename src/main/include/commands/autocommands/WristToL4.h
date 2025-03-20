#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakePitchSubsystem.h"

class WristToL4
  : public frc2::CommandHelper<frc2::Command, WristToL4> {
public:
  /**
   * Creates a new command WristToL4.
   *
   * @param intakepitch The pointer to the pitch subsystem
   */
  explicit WristToL4(IntakePitchSubsystem *intakepitch);

  void Initialize() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  IntakePitchSubsystem* m_intakePitch;
};
