// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.h"
#include <frc/system/plant/LinearSystemId.h>

SubDrivetrain::SubDrivetrain()
{
  mLeftMotorController = new rev::spark::SparkMax{CanIDConstants::kLeftCanID, rev::spark::SparkLowLevel::MotorType::kBrushless};
  mRightMotorController = new rev::spark::SparkMax{CanIDConstants::kRightCanID, rev::spark::SparkLowLevel::MotorType::kBrushless};

  mRightMotorController->SetInverted(true);

  //mSparkBaseConfig = new rev::spark::SparkBaseConfig;
  
  mDifferentialDrive = new frc::DifferentialDrive{*mLeftMotorController, *mRightMotorController};
  mGearBoxL = new frc::DCMotor{frc::DCMotor::NEO()};
  mGearBoxR = new frc::DCMotor{frc::DCMotor::NEO()};
  mRightMotorControllerSim = new rev::spark::SparkMaxSim{mRightMotorController, mGearBoxR};
  mLeftMotorControllerSim = new rev::spark::SparkMaxSim{mLeftMotorController, mGearBoxL};
  
  mDrivetrainSim = new frc::sim::DifferentialDrivetrainSim{frc::LinearSystemId::DrivetrainVelocitySystem(frc::DCMotor::NEO(2), 20_kg, 2_in, 20_cm, (20_kg * 15_in * 14_in) / 12, 1), 19_in, frc::DCMotor::NEO(2), 1, 2_in, {0, 0, 0, 0, 0, 0, 0}};

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
  //mSparkBaseConfig->OpenLoopRampRate(DriveConstants::kSpeedRampRate);
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