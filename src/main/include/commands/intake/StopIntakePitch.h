#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakePitchSubsystem.h"

class StopIntakePitch
  : public frc2::CommandHelper<frc2::Command, StopIntakePitch> {
public:
  /**
   * Creates a new StopIntakePitch.
   *
   * @param intakepitch The pointer to the pitch subsystem
   */
  explicit StopIntakePitch(IntakePitchSubsystem *intakepitch);

  void Initialize() override;
  void End(bool interrupted) override;
  
private:
  IntakePitchSubsystem* m_intakePitch;
};
