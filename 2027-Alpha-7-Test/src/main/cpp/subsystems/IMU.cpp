// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/IMU.hpp"

#include "Constants.hpp"

IMU::IMU()
{
	// mIMU = new ctre::phoenix6::hardware::Pigeon2{CANid::kIMUPigeonID};
	// mSimIMU = &mIMU->GetSimState();
}

void IMU::advanceSimulation(wpi::units::radians_per_second_t iYawVelocity)
{
	// mSimIMU->SetAngularVelocityX(0_rad_per_s);
	// mSimIMU->SetAngularVelocityY(0_rad_per_s);
	// mSimIMU->AddYaw(iYawVelocity * 0.02_s);
	// mSimIMU->SetAngularVelocityZ(iYawVelocity);
}

wpi::math::Rotation2d IMU::getRotation2d()
{
	return wpi::math::Rotation2d{0_deg};
	// return mIMU->GetRotation2d();
}

wpi::units::degree_t IMU::getAngleYaw()
{
	return 0_deg;
	// return mIMU->GetYaw().GetValue();
}

wpi::units::degrees_per_second_t IMU::getYawRate()
{
	return 0_deg_per_s;
	// return mIMU->GetAngularVelocityZWorld().GetValue();
}

void IMU::reset()
{
	// mIMU->Reset();
}

void IMU::setAngleYaw(wpi::units::degree_t iAngle)
{
	// mIMU->SetYaw(iAngle);
}

// void IMU::InitSendable(wpi::util::SendableBuilder& builder)
// {
// 	builder.SetSmartDashboardType("IMU");
// 	builder.AddDoubleProperty("rotation rads", [this] { return getRotation2d().Radians().value(); }, [this](double iValue) { setAngleYaw(wpi::units::radian_t(iValue)); });
// 	builder.AddDoubleProperty("rotation degrees", [this] { return getRotation2d().Degrees().value(); }, [this](double iValue) { setAngleYaw(wpi::units::degree_t(iValue)); });
// 	builder.AddDoubleProperty("angular velocity rad/sec", [this] { return wpi::units::radians_per_second_t(getYawRate()).value(); }, nullptr);
// 	builder.AddDoubleProperty("angular velocity deg/sec", [this] { return getYawRate().value(); }, nullptr);
// }
