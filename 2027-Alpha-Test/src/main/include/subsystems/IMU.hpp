// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include <ctre/phoenix6/Pigeon2.hpp>
// #include <ctre/phoenix6/sim/Pigeon2SimState.hpp>
#include <wpi/math/geometry/Rotation2d.hpp>
// #include <wpi/sendable/Sendable.h>
// #include <wpi/sendable/SendableBuilder.h>

#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>

// class IMU : public wpi::util::Sendable {
class IMU {
 public:
	IMU();

	wpi::math::Rotation2d getRotation2d();
	wpi::units::degree_t getAngleYaw();
	wpi::units::degrees_per_second_t getYawRate();

	void advanceSimulation(wpi::units::radians_per_second_t iYawVelocity);

	void reset();
	void setAngleYaw(wpi::units::degree_t iAngle);

	// void InitSendable(wpi::util::SendableBuilder& builder) override;

 private:
	// ctre::phoenix6::hardware::Pigeon2* mIMU;
	// ctre::phoenix6::sim::Pigeon2SimState* mSimIMU;
};
