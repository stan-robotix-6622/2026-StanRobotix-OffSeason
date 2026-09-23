#pragma once

#include <frc2/command/CommandPtr.h>
#include <stan/StanXboxController.h>

#include "Constants.h"
#include "subsystems/SubCarDrive.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  stan::StanXboxController mDriverController{
      OperatorConstants::kDriverControllerPort,
      OperatorConstants::kJoystickDeadband};

  SubCarDrive mCarDrive;

  void ConfigureBindings();
};
