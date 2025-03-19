#include "Constants.h"
#include "commands/autocommands/WristToIntake.h"

SetWristToIntake::SetWristToIntake(IntakePitchSubsystem *intakepitch) : m_intakePitch{intakepitch} {


  AddRequirements(m_intakePitch);
}

void SetWristToIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Initialized\r\n";
#endif
m_intakePitch->SetPitchPosition(IntakeConstants::kPitchToIntake);
}

bool SetWristToIntake::IsFinished(){
    if (abs(IntakeConstants::kPitchToIntake - m_intakePitch->GetCoralIntakeAngle()) < 0.001)
        return true;
    else
        return false; 
}

void SetWristToIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleIntake Ended\r\n";
#endif

}