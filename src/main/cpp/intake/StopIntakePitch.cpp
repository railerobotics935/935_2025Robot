
#include "Constants.h"
#include "commands/intake/StopIntakePitch.h"

StopIntakePitch::StopIntakePitch(IntakePitchSubsystem *intakepitch) : m_intakePitch{intakepitch} {


  AddRequirements(m_intakePitch);
}

void StopIntakePitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_intakePitch-> SetPitchPower(0.0);

}


void StopIntakePitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
}