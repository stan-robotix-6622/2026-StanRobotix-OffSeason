// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"
#include "Constants.h"

DriveTrain::DriveTrain() {
  mRightMotor = new ctre::phoenix6::hardware::TalonFX{DriveTrainConstants::kRightMotorID};
  mLeftMotor = new ctre::phoenix6::hardware::TalonFX{DriveTrainConstants::kLeftMotorID};
};

// This method will be called once per scheduler run
void DriveTrain::Periodic() {}

void DriveTrain::Drive(double RightY)
{
  // ctre::phoenix6::HootReplay::SetSpeed(RightY);
  mRightMotor->Set(RightY);
  mLeftMotor->Set(RightY);
}
