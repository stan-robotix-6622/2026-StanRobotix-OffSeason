// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Drive.h"

Drive::Drive(SubDrivetrain* iDrivetrain, double iTargetDistance) {
  // Use addRequirements() here to declare subsystem dependencies.
  mDrivetrain = iDrivetrain;
  mTargetDistance = iTargetDistance;
  mPIDController = new frc::PIDController{PIDConstants::kP, PIDConstants::kI, PIDConstants::kD};
  AddRequirements(mDrivetrain);
}

// Called when the command is initially scheduled.
void Drive::Initialize() 
{
  mDrivetrain->ResetEncoders();
  mPIDController->SetSetpoint(mTargetDistance);
}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() 
{
  double currentPosition = mDrivetrain->GetEncoderPosition();
  double speed = mPIDController->Calculate(currentPosition);
  mDrivetrain->DriveRobot(speed, speed);
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {
  mDrivetrain->Stop();
}

// Returns true when the command should end.
bool Drive::IsFinished() {
  return false;
}
