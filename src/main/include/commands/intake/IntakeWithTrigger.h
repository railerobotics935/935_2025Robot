#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>
#include <frc/MathUtil.h>

#include "subsystems/IntakeSubsystem.h"

class IntakeWithTrigger
  : public frc2::CommandHelper<frc2::Command, IntakeWithTrigger> {
public:
  /**
   * Creates a new IntakeWithTrigger.
   *
   * @param intake The pointer to the intake subsystem
   */
  explicit IntakeWithTrigger(IntakeSubsystem *intake, frc::XboxController *operatorController);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  
private:
  IntakeSubsystem* m_intake;
  frc::XboxController* m_operatorController;
};
