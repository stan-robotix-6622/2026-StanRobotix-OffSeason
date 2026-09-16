// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "Constants.h"

Drivetrain::Drivetrain()
{
  mLeftMotorController = new ctre::phoenix6::hardware::TalonFX{CanIDConstants::MotorLeftID};
  mRightMotorController = new ctre::phoenix6::hardware::TalonFX{CanIDConstants::MotorRightID};
};

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}
