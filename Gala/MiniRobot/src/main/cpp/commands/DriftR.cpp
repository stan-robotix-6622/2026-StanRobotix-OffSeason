// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriftR.h"

DriftR::DriftR(SubDrivetrain* iDriveTrain, std::function<double()> iSpeed) {
  // Use addRequirements() here to declare subsystem dependencies.
  mDrivetrain = iDriveTrain;
  AddRequirements(mDrivetrain);
  mSpeed = iSpeed;
}

// Called when the command is initially scheduled.
void DriftR::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriftR::Execute() 
{
	for (float i = 0; i < 0.1; i+= 0.01)
	{
		mDrivetrain->DriveRobot(mSpeed(), i);
	}
}

// Called once the command ends or is interrupted.
void DriftR::End(bool interrupted) {}

// Returns true when the command should end.
bool DriftR::IsFinished() {
  return false;
}
