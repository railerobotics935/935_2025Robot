#include "Constants.h"
#include "commands/autocommands/WristToL4.h"

WristToL4::WristToL4(IntakePitchSubsystem *intakepitch) : m_intakePitch{intakepitch} {


  AddRequirements(m_intakePitch);
}

void WristToL4::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_intakePitch->SetPitchPosition(IntakeConstants::kPitchL4);
}

bool WristToL4::IsFinished(){
    if (abs(IntakeConstants::kPitchL4 - m_intakePitch->GetIntakeAngle()) < 0.001)
        return true;
    else
        return false; 
}

void WristToL4::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}