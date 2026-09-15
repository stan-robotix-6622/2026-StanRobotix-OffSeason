// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

#include "Constants.h"

RobotContainer::RobotContainer()
{
  mDriverController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};

  mDrivetrain = new SubDrivetrain{};

  mDrivetrain->SetDefaultCommand(mDrivetrain->getDriveCommand(
    [this] {return mDriverController->GetLeftX();},
    [this] {return mDriverController->GetLeftY();},
    [this] {return mDriverController->GetRightX();},
    [this] {return 1 - mDriverController->GetRightTriggerAxis();},
    true));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
