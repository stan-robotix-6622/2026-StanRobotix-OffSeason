#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/SubCarDrive.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc2::CommandXboxController mDriverController{OperatorConstants::kDriverControllerPort};

  SubCarDrive mCarDrive;

  void ConfigureBindings();
};
