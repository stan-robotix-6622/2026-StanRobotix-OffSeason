#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::AutonomousInit() {
  mAutonomousCommand = mContainer.GetAutonomousCommand();

  if (mAutonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(mAutonomousCommand.value());
  }
}

void Robot::TeleopInit() {
  if (mAutonomousCommand) {
    mAutonomousCommand->Cancel();
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
