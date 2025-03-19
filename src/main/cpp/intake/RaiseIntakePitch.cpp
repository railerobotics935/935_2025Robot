
#include "Constants.h"
#include "commands/intake/RaiseIntakePitch.h"

RaiseIntakePitch::RaiseIntakePitch(IntakePitchSubsystem *intakepitch) : m_intakePitch{intakepitch} {


  AddRequirements(m_intakePitch);
}

void RaiseIntakePitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_intakePitch-> SetPitchPower(-0.3);
}

void RaiseIntakePitch::Execute() {
  m_intakePitch-> SetPitchPower(-0.3);
}

void RaiseIntakePitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_intakePitch-> SetPitchPower(0.0);
}