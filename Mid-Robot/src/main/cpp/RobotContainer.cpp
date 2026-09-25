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
    double steer = frc::ApplyDeadband(mDriverController.GetLeftX(), OperatorConstants::kJoystickDeadband);
    bool reverse = mDriverController.GetHID().GetBButton();
    bool drift = mDriverController.LeftBumper().Get();
    mCarDrive.drive(throttle, brake, steer, reverse, drift);
  }));

  mDriverController.A().WhileTrue(mCarDrive.getTestSteerCommand());
  mDriverController.Y().OnTrue(mCarDrive.getCalibrateRatioCommand());
  mDriverController.POVLeft().OnTrue(mCarDrive.getZeroFLCommand());
  mDriverController.POVRight().OnTrue(mCarDrive.getZeroFRCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
