
#include "Constants.h"
#include "commands/intake/SimpleOuttake.h"

SimpleOuttake::SimpleOuttake(IntakeSubsystem *intake) : m_intake{intake} {

  AddRequirements(m_intake);
}

void SimpleOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(-0.3);
}


void SimpleOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
}