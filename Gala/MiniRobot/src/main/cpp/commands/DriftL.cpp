// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriftL.h"

DriftL::DriftL(SubDrivetrain* iDriveTrain, std::function<double()> iSpeed) {
  // Use addRequirements() here to declare subsystem dependencies.
  mDrivetrain = iDriveTrain;
  AddRequirements(mDrivetrain);
  mSpeed = iSpeed;
}

// Called when the command is initially scheduled.
void DriftL::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriftL::Execute() 
{
  mDrivetrain->DriveRobot(mSpeed(), -0.03);
}

// Called once the command ends or is interrupted.
void DriftL::End(bool interrupted) {}

// Returns true when the command should end.
bool DriftL::IsFinished() {
  return false;
}
