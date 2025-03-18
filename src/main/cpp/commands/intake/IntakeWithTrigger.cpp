
#include "Constants.h"
#include "commands/intake/IntakeWithTrigger.h"

IntakeWithTrigger::IntakeWithTrigger(IntakeSubsystem *intake, frc::XboxController *operatorController) : m_intake{intake}, m_operatorController{operatorController} {

  AddRequirements(m_intake);
}

void IntakeWithTrigger::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "IntakeWithTrigger Initialized\r\n";
#endif
}

void IntakeWithTrigger::Execute() {
    
  const auto intakeSpeed = -frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kIntakeTriggerIndex), 0.05);
  const auto outtakeSpeed = frc::ApplyDeadband(m_operatorController->GetRawAxis(ControllerConstants::kOuttakeTriggerIndex), 0.05);

  if (intakeSpeed < 0) {
    m_intake->SetIntakeMotorPower(m_intake->SignedSquare(intakeSpeed));
  }
  else if (outtakeSpeed > 0) {
    m_intake->SetIntakeMotorPower(m_intake->SignedSquare(outtakeSpeed));
  }
  else {
    m_intake->SetIntakeMotorPower(0.0);
  }
}

void IntakeWithTrigger::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "IntakeWithTrigger Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
}