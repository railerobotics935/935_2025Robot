
#include "Constants.h"
#include "commands/intake/RaiseIntakePitch.h"

RaiseIntakePitch::RaiseIntakePitch(IntakePitchSubsystem *intakepitch) : m_intakePitch{intakepitch} {
  AddRequirements(m_intakePitch);
}

void RaiseIntakePitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
    currentIntakeAngle = m_intakePitch->GetIntakeAngle();
}

void RaiseIntakePitch::Execute() {
  currentIntakeAngle -= 0.004;
  m_intakePitch->SetIntakeAngle(currentIntakeAngle);
}

void RaiseIntakePitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}