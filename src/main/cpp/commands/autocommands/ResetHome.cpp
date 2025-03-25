#include "Constants.h"
#include "commands/autocommands/ResetHome.h"

ResetHome::ResetHome(FourBarSubsystem* fourbar, IntakePitchSubsystem* pitch) : m_fourBar{fourbar}, m_pitch {pitch} {
  AddRequirements(m_fourBar);
  AddRequirements(m_pitch);
}

void ResetHome::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "ResetHome Initialized\r\n";
#endif

  m_fourBar->SetFourBarHeight(FourBarConstants::kMinimumHeight);
  m_pitch->SetPitchPosition(IntakeConstants::kMinimumAngle);

}

bool ResetHome::IsFinished(){
    if (abs(FourBarConstants::kMinimumHeight - m_fourBar->GetFourBarHeight()) < 0.001)
      return true;
    else 
        return false;

}

void ResetHome::End(bool interrupted) {

}