
#include "Constants.h"
#include "commands/autocommands/SimpleIntake.h"

SimpleIntake::SimpleIntake(IntakeSubsystem *intake) : m_intake{intake} {

  m_intake = intake;

  AddRequirements(m_intake);
}

void SimpleIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
  m_intake->SetIntakeMotorPower(-0.5);

}


bool SimpleIntake::IsFinished() {
  if(m_intake->CoralInIntake() && m_intake->GetDirection() < 0) {
    return true;
  }
  else {
    return false;
  }
}


void SimpleIntake::End(bool interrupted) {
  m_intake->SetIntakeMotorPower(0.0);
  //m_coralIntake->SetCoralIntakeAngle(0.0);
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}