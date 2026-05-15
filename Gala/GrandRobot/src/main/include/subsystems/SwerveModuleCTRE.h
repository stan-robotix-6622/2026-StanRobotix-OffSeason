// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Constants.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/sim/TalonFXSimState.hpp>

// #include <rev/sim/SparkMaxSim.h>
// #include <rev/SparkAbsoluteEncoder.h>
// #include <rev/SparkClosedLoopController.h>
// #include <rev/SparkMax.h>
// #include <rev/SparkRelativeEncoder.h>
// #include <wpi/sendable/Sendable.h>
// #include <wpi/sendable/SendableBuilder.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <ctre/phoenix6/TalonFX.hpp>

class SwerveModule : public wpi::Sendable {
 public:
	SwerveModule(int iDrivingMotorID, int iTurningMotorID, int iTurningCANcoderID, bool iDrivingInveryed = false, bool iTurningInverted = true);

	frc::SwerveModulePosition getModulePosition();
	frc::SwerveModuleState getModuleState();
	

	units::radians_per_second_t getTurningVelocity();

	void InitSendable(wpi::SendableBuilder& builder) override;

	void setDesiredState(frc::SwerveModuleState iDesiredState);
	void setDesiredHeading(frc::Rotation2d iDesiredHeading);

	void setTurningVoltage(units::volt_t iVoltage);
	void setDrivingVoltage(units::volt_t iVoltage);

	void seedEncoder();
	void refreshModule();

 private:
	// rev::spark::SparkMax* mDrivingMotor;
	// rev::spark::SparkMax* mTurningMotor;
	ctre::phoenix6::hardware::TalonFX * mDrivingMotor;
	ctre::phoenix6::hardware::TalonFX * mTurningMotor;

	// rev::spark::SparkClosedLoopController* mDrivingClosedLoopController;
	frc::SimpleMotorFeedforward<units::meter>* mDrivingFeedforward;
	frc::PIDController * mTurningPIDController;

	ctre::phoenix6::hardware::CANcoder * mTurningCANcoder;

	// For simulation
	bool mRobotIsSimulated = false;
	frc::DCMotor * mDrivingGearBox;
	frc::DCMotor * mTurningGearBox;
	// rev::spark::SparkMaxSim* mDrivingMotorSim;
	// rev::spark::SparkMaxSim* mTurningMotorSim;
	
	ctre::phoenix6::sim::TalonFXSimState* mDrivingMotorSim;
	ctre::phoenix6::sim::TalonFXSimState* mTurningMotorSim;

	frc::Rotation2d mTurningCurrentAngle;

	frc::SwerveModuleState mOptimizedState;

	frc::SwerveModuleState mModuleState;
	frc::SwerveModulePosition mModulePosition;
};
