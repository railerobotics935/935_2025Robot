#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakePitchSubsystem.h"

class StopIntakePitch
  : public frc2::CommandHelper<frc2::Command, StopIntakePitch> {
public:
  /**
   * Creates a new StopIntakePitch.
   *
   * @param intakePitch The pointer to the intakePitch subsystem
   */
  explicit StopIntakePitch(IntakePitchSubsystem *intake);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakePitchSubsystem* m_intakePitch;
  double currentIntakeAngle;
};
