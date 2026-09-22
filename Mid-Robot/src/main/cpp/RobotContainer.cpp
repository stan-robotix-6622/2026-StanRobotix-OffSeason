#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  mCarDrive.SetDefaultCommand(mCarDrive.Run([this] {
    double throttle = mDriverController.GetRightTriggerAxis();
    double brake = mDriverController.GetLeftTriggerAxis();
    double steer = frc::ApplyDeadband(mDriverController.GetRightX(), OperatorConstants::kJoystickDeadband);
    mCarDrive.drive(throttle, brake, steer, mDriverController.GetHID().GetBButton());
  }));

  mDriverController.A().WhileTrue(mCarDrive.getTestSteerCommand());
  mDriverController.LeftBumper().OnTrue(mCarDrive.getZeroFLCommand());
  mDriverController.RightBumper().OnTrue(mCarDrive.getZeroFRCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
