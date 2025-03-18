
#include "Constants.h"
#include "commands/intake/LowerIntakePitch.h"

LowerIntakePitch::LowerIntakePitch(IntakePitchSubsystem *intakepitch) : m_intakePitch{intakepitch} {
  AddRequirements(m_intakePitch);
}

void LowerIntakePitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
    currentIntakeAngle = m_intakePitch->GetIntakeAngle();
}

void LowerIntakePitch::Execute() {
  currentIntakeAngle += 0.004;
  m_intakePitch->SetIntakeAngle(currentIntakeAngle);
}

void LowerIntakePitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}