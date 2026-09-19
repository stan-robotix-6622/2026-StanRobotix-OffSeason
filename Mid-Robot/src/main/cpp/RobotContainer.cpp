#include "RobotContainer.h"

#include <frc/MathUtil.h>
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  mCarDrive.SetDefaultCommand(mCarDrive.Run([this] {
    double wThrottle = m_driverController.GetRightTriggerAxis();
    double wBrake = m_driverController.GetLeftTriggerAxis();
    double wSteer = frc::ApplyDeadband(m_driverController.GetRightX(), OperatorConstants::kJoystickDeadband);
    mCarDrive.drive(wThrottle, wBrake, wSteer);
  }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::None();
}
