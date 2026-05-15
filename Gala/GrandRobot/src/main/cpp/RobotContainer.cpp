// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  mSubDriveTrain = new SubDrivetrain;
  mSubIMU = new IMU;
}

void RobotContainer::ConfigureBindings() {
  mSubDriveTrain->SetDefaultCommand(frc2::cmd::Run([this]{mSubDriveTrain->driveRobotRelative(0)};));
  frc2::Trigger([this]);
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
