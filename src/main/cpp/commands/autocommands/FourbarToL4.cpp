
#include "Constants.h"
#include "commands/autocommands/FourBarToL4.h"
FourBarToL4::FourBarToL4(FourBarSubsystem* fourBar) : m_fourBar{fourBar} {
  AddRequirements(m_fourBar);

  // Set the distance per pulse if needed
//  m_elevatorEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
}

void FourBarToL4::Execute() {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Initialized\r\n";
#endif

  m_fourBar->SetFourBarHeight(FourBarConstants::kFourBarL4);
  
 
}

bool FourBarToL4::IsFinished(){
  if (abs(FourBarConstants::kFourBarL4 - m_fourBar->GetFourBarHeight()) < 0.001)
    return true;
  else
    return false;

}
void FourBarToL4::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Ended\r\n";
#endif

  // m_fourBar->SetFourBarPower(0.0);
}