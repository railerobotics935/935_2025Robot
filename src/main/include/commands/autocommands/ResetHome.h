#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/FourBarSubsystem.h"
#include "subsystems/IntakePitchSubsystem.h"

class ResetHome
  : public frc2::CommandHelper<frc2::Command, ResetHome> {
public:
  /**
   * Creates a new Reset home command.
   *
   * @param fourbar The pointer to the fourbar subsystem
   * @param pitch The pointer to the coral pitch subsystem
   */
  explicit ResetHome(FourBarSubsystem* fourbar, IntakePitchSubsystem* pitch);

  void Initialize() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  
private:
  FourBarSubsystem* m_fourBar;
  IntakePitchSubsystem* m_pitch;
};
