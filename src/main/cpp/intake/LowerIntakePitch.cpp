
#include "Constants.h"
#include "commands/intake/LowerIntakePitch.h"

LowerIntakePitch::LowerIntakePitch(IntakePitchSubsystem *intakepitch) : m_intakePitch{intakepitch} {


  AddRequirements(m_intakePitch);
}

void LowerIntakePitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_intakePitch->SetPitchPower(0.15);
}

void LowerIntakePitch::Execute() {
  m_intakePitch->SetPitchPower(0.15);
}

void LowerIntakePitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
m_intakePitch-> SetPitchPower(0.0);
}