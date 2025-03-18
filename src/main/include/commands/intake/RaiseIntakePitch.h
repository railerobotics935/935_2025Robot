#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakePitchSubsystem.h"

class RaiseIntakePitch
  : public frc2::CommandHelper<frc2::Command, RaiseIntakePitch> {
public:
  /**
   * Creates a new RaiseIntakePitch.
   *
   * @param intakePitch The pointer to the intakePitch subsystem
   */
  explicit RaiseIntakePitch(IntakePitchSubsystem *intake);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  IntakePitchSubsystem* m_intakePitch;
  double currentIntakeAngle;
};
