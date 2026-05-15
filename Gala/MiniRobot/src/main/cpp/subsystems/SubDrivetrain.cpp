// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.h"

SubDrivetrain::SubDrivetrain()
{
  mLeftMotorController = new rev::spark::SparkMax{CanIDConstants::kLeftCanID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  mRightMotorController = new rev::spark::SparkMax{CanIDConstants::kRightCanID, rev::spark::SparkLowLevel::MotorType::kBrushless};

  mRightMotorController->SetInverted(true);

  mDifferentialDrive = new frc::DifferentialDrive{*mLeftMotorController, *mRightMotorController};
}
// This method will be called once per scheduler run
void SubDrivetrain::Periodic() {}

// units::meters_per_second_t SubDrivetrain::getSpeed()
// {
//   return mDifferentialDrive->;
// }

// void SubDrivetrain::setSpeed()
// {

// }

void SubDrivetrain::DriveRobot(double iLSpeed, double iRSpeed)
{
  mDifferentialDrive->TankDrive(iLSpeed, iRSpeed, true);
}

double SubDrivetrain::GetEncoderPosition()
{
  return mLeftMotorController->GetEncoder().GetPosition();
}

void SubDrivetrain::ResetEncoders()
{
  mLeftMotorController->GetEncoder().SetPosition(0);
  mRightMotorController->GetEncoder().SetPosition(0);
}

void SubDrivetrain::Stop()
{
  mDifferentialDrive->TankDrive(0.0, 0.0);
}