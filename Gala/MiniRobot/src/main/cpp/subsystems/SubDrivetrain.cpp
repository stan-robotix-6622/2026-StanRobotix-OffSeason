// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.h"

#include <rev/config/SparkMaxConfig.h>

SubDrivetrain::SubDrivetrain()
{
  mLeftMotorController = new rev::spark::SparkMax{CanIDConstants::kLeftCanID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  mRightMotorController = new rev::spark::SparkMax{CanIDConstants::kRightCanID, rev::spark::SparkLowLevel::MotorType::kBrushless};

<<<<<<< HEAD
  mRightMotorController->Configure(rev::spark::SparkMaxConfig{}.Inverted(true), rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);

=======
  mRightMotorController->SetInverted(true);
  
>>>>>>> fc0c6ed111b17d4e621f6d159f8195a26afc2d1d
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

void SubDrivetrain::DriveRobot(double iSpeed, double iRotation)
{
  mDifferentialDrive->ArcadeDrive(iSpeed, iRotation, true);
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
  mDifferentialDrive->ArcadeDrive(0.0, 0.0);
}