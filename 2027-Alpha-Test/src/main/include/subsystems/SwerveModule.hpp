// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/math/geometry/Rotation2d.hpp>
#include <wpi/math/controller/PIDController.hpp>
#include <wpi/math/controller/SimpleMotorFeedforward.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/simulation/FlywheelSim.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <rev/sim/SparkMaxSim.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
// #include <wpi/sendable/Sendable.h>
// #include <wpi/sendable/SendableBuilder.h>

#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

// class SwerveModule : public wpi::util::Sendable {
class SwerveModule {
 public:
	SwerveModule(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInveryed = false, bool iTurningInverted = true);

	wpi::math::SwerveModulePosition getModulePosition();
	wpi::math::SwerveModuleVelocity getModuleVelocity();

	wpi::units::radians_per_second_t getTurningVelocity();

	// void InitSendable(wpi::util::SendableBuilder& builder) override;

	void setDesiredState(wpi::math::SwerveModuleVelocity iDesiredState);
	void setDesiredHeading(wpi::math::Rotation2d iDesiredHeading);

	void setDrivingVoltage(wpi::units::volt_t iVoltage);
	void setTurningVoltage(wpi::units::volt_t iVoltage);

	void seedEncoder();
	void refreshModule();

 private:
	rev::spark::SparkMax* mDrivingMotor;
	rev::spark::SparkMax* mTurningMotor;

	rev::spark::SparkClosedLoopController* mDrivingClosedLoopController;
	rev::spark::SparkClosedLoopController* mTurningClosedLoopController;

	rev::spark::SparkRelativeEncoder* mDrivingEncoder;
	rev::spark::SparkRelativeEncoder* mTurningEncoder;
	rev::spark::SparkAbsoluteEncoder* mTurningAbsoluteEncoder;

	// For simulation
	wpi::math::DCMotor* mDrivingGearBox;
	wpi::math::DCMotor* mTurningGearBox;
	rev::spark::SparkMaxSim* mDrivingMotorSim;
	rev::spark::SparkMaxSim* mTurningMotorSim;

	wpi::sim::FlywheelSim* mDrivingFlywheelSim;
	wpi::sim::FlywheelSim* mTurningFlywheelSim;

	wpi::math::SimpleMotorFeedforward<wpi::units::meters>* mDrivingFeedforward;
	wpi::math::SimpleMotorFeedforward<wpi::units::radians>* mTurningFeedforward;
	wpi::math::PIDController* mDrivingPID;
	wpi::math::PIDController* mTurningPID;

	wpi::math::Rotation2d mTurningCurrentAngle;

	wpi::math::SwerveModuleVelocity mOptimizedState;

	wpi::math::SwerveModuleVelocity mModuleVelocity;
	wpi::math::SwerveModulePosition mModulePosition;
};
