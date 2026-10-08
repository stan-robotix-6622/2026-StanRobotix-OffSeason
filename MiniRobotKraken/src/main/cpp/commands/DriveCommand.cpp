// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommand.h"


DriveCommand::DriveCommand(DriveTrain *iDriveCommand, frc2::CommandXboxController* iXboxController, double iTargetDistance)
{
 mDriveCommand = iDriveCommand;
 mXboxController = iXboxController;
 mTargetDistance = iTargetDistance;
 AddRequirements(iDriveCommand);

 mPIDController = new frc::PIDController{PIDConstants::kP, PIDConstants::kI, PIDConstants::kD};
}
// Called when the command is initially scheduled.
void DriveCommand::Initialize()
{
	mPIDController->SetSetpoint(mTargetDistance);
}

// Called repeatedly when this Command is scheduled to run
void DriveCommand::Execute() 
{
  
}

// Called once the command ends or is interrupted.
void DriveCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveCommand::IsFinished() {
  return false;
}

