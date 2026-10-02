// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveTrain.h"
#include "Constants.h"

DriveTrain::DriveTrain() {
  mRightMotorDrive = new ctre::phoenix6::hardware::TalonFX{DriveTrainConstants::kRightMotorDriveID};
  mLeftMotorDrive = new ctre::phoenix6::hardware::TalonFX{DriveTrainConstants::kLeftMotorDriveID};
	mRightMotorTurn = new ctre::phoenix6::hardware::TalonFX{DriveTrainConstants::kRightMotorTurnID};
	mLeftMotorTurn = new ctre::phoenix6::hardware::TalonFX{DriveTrainConstants::kLeftMotorTurnID};
};

// This method will be called once per scheduler run
void DriveTrain::Periodic() {}

void DriveTrain::Drive(double Drive, double Turn)
{
  // ctre::phoenix6::HootReplay::SetSpeed(RightY);
  mRightMotorDrive->Set(Drive);
  mRightMotorTurn->Set(Turn);

  mLeftMotorDrive->Set(Drive);
	mLeftMotorTurn->Set(Turn);
}
