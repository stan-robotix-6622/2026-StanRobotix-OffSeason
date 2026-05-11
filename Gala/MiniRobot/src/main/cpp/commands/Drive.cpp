// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Drive.h"

Drive::Drive(SubDrivetrain* iDrivetrain) {
  // Use addRequirements() here to declare subsystem dependencies.
  mDrivetrain = new SubDrivetrain;
  mPIDController = new frc::PIDController{PIDConstants::kP, PIDConstants::kI, PIDConstants::kD};
  AddRequirements(mDrivetrain);
}

// Called when the command is initially scheduled.
void Drive::Initialize() 
{
  mPIDController->SetSetpoint(DriveConstants::kSpeed);
}

// Called repeatedly when this Command is scheduled to run
void Drive::Execute() 
{
  mDrivetrain->Drive(mPIDController->Calculate(DriveConstants::kSpeed), mPIDController->Calculate(DriveConstants::kSpeed));
}

// Called once the command ends or is interrupted.
void Drive::End(bool interrupted) {}

// Returns true when the command should end.
bool Drive::IsFinished() {
  return false;
}
