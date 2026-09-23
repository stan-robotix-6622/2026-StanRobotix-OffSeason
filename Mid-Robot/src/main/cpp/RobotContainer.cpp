#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  mCarDrive.SetDefaultCommand(mCarDrive.getDriveCommand(
      [this] { return mDriverController.getRightTriggerWithDeadband(); },
      [this] { return mDriverController.getLeftTriggerWithDeadband(); },
      [this] { return mDriverController.getRightXWithDeadband(); },
      [this] { return mDriverController.GetHID().GetBButton(); }));

  mDriverController.bindHold(mDriverController.A(), mCarDrive.getTestSteerCommand());
  mDriverController.bindPress(mDriverController.LeftBumper(), mCarDrive.getZeroFLCommand());
  mDriverController.bindPress(mDriverController.RightBumper(), mCarDrive.getZeroFRCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
