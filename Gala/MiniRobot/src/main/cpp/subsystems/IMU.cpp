// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IMU.h"

IMU::IMU()
{
	mIMU = new ctre::phoenix6::hardware::Pigeon2{CanIDConstants::kIMUCanID};
}

frc::Rotation2d IMU::getRotation2d()
{
	return mIMU->GetRotation2d();
}

units::degree_t IMU::getAngleYaw()
{
	return mIMU->GetYaw().GetValue();
}

units::degrees_per_second_t IMU::getYawRate()
{
	return mIMU->GetAngularVelocityZWorld().GetValue();
}

void IMU::reset()
{
	mIMU->Reset();
}

void IMU::setAngleYaw(units::degree_t iAngle)
{
	mIMU->SetYaw(iAngle);
}

// This method will be called once per scheduler run
void IMU::Periodic() {}
