#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousInit() {
  mAutonomousCommand = mContainer.GetAutonomousCommand();

  if (mAutonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(mAutonomousCommand.value());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  if (mAutonomousCommand) {
    mAutonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
