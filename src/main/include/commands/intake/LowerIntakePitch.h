#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakePitchSubsystem.h"

class LowerIntakePitch
  : public frc2::CommandHelper<frc2::Command, LowerIntakePitch> {
public:
  /**
   * Creates a new LowerIntakePitch.
   *
   * @param intakepitch The pointer to the pitch subsystem
   */
  explicit LowerIntakePitch(IntakePitchSubsystem *intakepitch);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  IntakePitchSubsystem* m_intakePitch;
  double currentIntakeAngle;
};
