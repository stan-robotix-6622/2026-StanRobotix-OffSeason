// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModuleCTRE.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>


SwerveModule::SwerveModule(int iDrivingMotorID, int iTurningMotorID, int iTurningCANcoderID, bool iDrivingInverted, bool iTurningInverted)
{
	mDrivingMotor = new ctre::phoenix6::hardware::TalonFX{iDrivingMotorID};
	mTurningMotor = new ctre::phoenix6::hardware::TalonFX{iTurningMotorID};

	// Simulation
	if (frc::RobotBase::IsSimulation()) {
		mRobotIsSimulated = true;
		mTurningGearBox = new frc::DCMotor{frc::DCMotor::KrakenX60()};
		mDrivingGearBox = new frc::DCMotor{frc::DCMotor::KrakenX60()};
		// mTurningMotorSim = new rev::spark::SparkMaxSim{mTurningMotor, mTurningGearBox};
		// mDrivingMotorSim = new rev::spark::SparkMaxSim{mDrivingMotor, mDrivingGearBox};
		mTurningMotorSim = new ctre::phoenix6::sim::TalonFXSimState{iTurningMotorID};
		mDrivingMotorSim = new ctre::phoenix6::sim::TalonFXSimState{iDrivingMotorID};
	}

	// mDrivingMotor->Configure(Configs::SwerveModule::DrivingConfig(iDrivingInverted),
	//                          ModuleConstants::kDrivingResetMode,
	//                          ModuleConstants::kDrivingPersistMode);
	// mTurningMotor->Configure(Configs::SwerveModule::TurningConfig(iTurningInverted),
	//                          ModuleConstants::kTurningResetMode,
	//                          ModuleConstants::kTurningPersistMode);

	// Initialization of the motors' ClosedLoopController
	// mDrivingClosedLoopController = new rev::spark::SparkClosedLoopController{mDrivingMotor->GetClosedLoopController()};
	mDrivingFeedforward = new frc::SimpleMotorFeedforward<units::meter>{0_V, ModuleConstants::kNominalVoltage / ModuleConstants::kDriveWheelMaxFreeSpeed};
	mTurningPIDController = new frc::PIDController{ModuleConstants::kTurningP, ModuleConstants::kTurningI, ModuleConstants::kTurningD};
	mTurningPIDController->SetTolerance(ModuleConstants::Config::kTurningClosedLoopTolerance);
	mTurningPIDController->EnableContinuousInput(ModuleConstants::Config::kTurningClosedLoopMinInput, ModuleConstants::Config::kTurningClosedLoopMaxInput);

	mTurningCANcoder = new ctre::phoenix6::hardware::CANcoder{iTurningCANcoderID};

	refreshModule();
	seedEncoder();
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState)
{
	mTurningCurrentAngle = frc::Rotation2d(mTurningMotor->GetPosition().GetValue());
	mOptimizedState = iDesiredState;
	mOptimizedState.Optimize(mTurningCurrentAngle);
	mOptimizedState.CosineScale(mTurningCurrentAngle);

	// mDrivingClosedLoopController->SetSetpoint(mOptimizedState.speed.value(), ModuleConstants::kDrivingClosedLoopControlType);
	mTurningPIDController->SetSetpoint(mOptimizedState.angle.Radians().value());

	// if (mRobotIsSimulated) {
	// 	mTurningMotorSim->iterate((mOptimizedState.angle.Radians().value() - mTurningMotorSim->GetPosition()) / 0.02, 12, 0.02);
	// 	mDrivingMotorSim->iterate(mOptimizedState.speed.value(), 12, 0.02);
	// }
}

void SwerveModule::setDesiredHeading(frc::Rotation2d iDesiredHeading)
{
	mTurningPIDController->SetSetpoint(iDesiredHeading.Radians().value());

	// if (mRobotIsSimulated) {
	// 	mTurningMotorSim->iterate((iDesiredHeading.Radians().value() - mTurningMotorSim->GetPosition()) / 0.02, 12, 0.02);
	// }
}

void SwerveModule::setTurningVoltage(units::volt_t iVoltage)
{
	mTurningMotor->SetVoltage(iVoltage);
}

void SwerveModule::setDrivingVoltage(units::volt_t iVoltage)
{
	mDrivingMotor->SetVoltage(iVoltage);
}

frc::SwerveModuleState SwerveModule::getModuleState()
{
	return mModuleState;
}

frc::SwerveModulePosition SwerveModule::getModulePosition()
{
	return mModulePosition;
}

units::radians_per_second_t SwerveModule::getTurningVelocity()
{
  return mTurningMotor->GetVelocity().GetValue();
}

void SwerveModule::seedEncoder()
{
	mTurningMotor->SetPosition(mTurningCANcoder->GetAbsolutePosition().GetValue());
}

void SwerveModule::refreshModule()
{
	mModuleState = frc::SwerveModuleState{mDrivingMotor->GetVelocity().GetValue() * ModuleConstants::kWheelRadius * 2 * std::numbers::pi / 1_tr * ModuleConstants::kDrivingMotorGearRatio,
																				mTurningMotor->GetPosition().GetValue() * ModuleConstants::kTurningGearRatio};

	mModulePosition = frc::SwerveModulePosition{mDrivingMotor->GetPosition().GetValue() * ModuleConstants::kWheelRadius * 2 * std::numbers::pi / 1_tr * ModuleConstants::kDrivingMotorGearRatio,
																							mTurningMotor->GetPosition().GetValue() * ModuleConstants::kTurningGearRatio};
}

void SwerveModule::InitSendable(wpi::SendableBuilder& builder)
{
	// builder.SetSmartDashboardType("swerve/module");
	// builder.AddDoubleProperty("turning velocity", [this] { return mTurningEncoder->GetVelocity(); }, nullptr);
	// builder.AddDoubleProperty("turning position", [this] { return mTurningEncoder->GetPosition(); }, nullptr);
	// builder.AddDoubleProperty("driving velocity", [this] { return mDrivingEncoder->GetVelocity(); }, nullptr);
}
