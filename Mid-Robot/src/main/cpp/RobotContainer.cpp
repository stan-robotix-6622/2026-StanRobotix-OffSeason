#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  mCarDrive.SetDefaultCommand(mCarDrive.Run([this] {
    double throttle = m_driverController.GetRightTriggerAxis();
    double brake = m_driverController.GetLeftTriggerAxis();
    double steer = frc::ApplyDeadband(m_driverController.GetRightX(), OperatorConstants::kJoystickDeadband);
    mCarDrive.drive(throttle, brake, steer);
  }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
