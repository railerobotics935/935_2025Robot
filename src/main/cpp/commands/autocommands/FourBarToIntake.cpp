
#include "Constants.h"
#include "commands/autocommands/FourBarToIntake.h"
FourBarToIntake::FourBarToIntake(FourBarSubsystem* fourBar) : m_fourBar{fourBar} {
  AddRequirements(m_fourBar);

  // Set the distance per pulse if needed
//  m_elevatorEncoder.SetDistancePerPulse(0.02 / 360.0); // Example for a 360 PPR encoder
}

void FourBarToIntake::Execute() {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Initialized\r\n";
#endif

  m_fourBar->SetFourBarHeight(FourBarConstants::kFourBarIntake);
  
 
}

bool FourBarToIntake::IsFinished(){
  if (abs(FourBarConstants::kFourBarIntake - m_fourBar->GetFourBarHeight()) < 0.001)
    return true;
  else
    return false;

}
void FourBarToIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "ElevatorSetPoint Ended\r\n";
#endif

  // m_fourBar->SetFourBarPower(0.0);
}