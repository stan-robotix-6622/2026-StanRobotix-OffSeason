#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/DriveTrain.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();

  frc2::CommandXboxController mDriverController{OperatorConstants::kDriverControllerPort};
  DriveTrain mDriveTrain;
};
