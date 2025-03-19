
#include "commands/autocommands/SimpleOuttake.h"


SimpleOuttake::SimpleOuttake(IntakeSubsystem* intake) : m_intake{intake} {
  // Initilize local copys of pointers

  // Add reqierments for the command
  AddRequirements(m_intake);
}

void SimpleOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.5);
  
}

void SimpleOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleOuttake Ended\r\n";
#endif
  m_intake->SetIntakeMotorPower(0.0);
  //m_coralIntake->SetCoralIntakeAngle(0.0);

}