
#include "Constants.h"
#include "commands/intake/SimpleIntake.h"

SimpleIntake::SimpleIntake(IntakeSubsystem *intake) : m_intake{intake} {

  AddRequirements(m_intake);
}

void SimpleIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.3);
}


void SimpleIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
}