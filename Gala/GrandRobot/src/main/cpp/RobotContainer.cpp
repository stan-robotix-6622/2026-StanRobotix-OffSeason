// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() 
{
  mSubDriveTrain = new SubDrivetrain;
  mSubIMU = new IMU;
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  mSubDriveTrain->SetDefaultCommand(mSubDriveTrain->Run
    ([this]{mSubDriveTrain->driveFieldRelative
      ( mSubDriveTrain->getPose().X().value(),
        mSubDriveTrain->getPose().Y().value(),
        mSubDriveTrain->getIMU()->getRotation2d().Degrees().value(),
        mSubDriveTrain->getIMU()->getYawRate().value());})); //speed modulation to fix
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
