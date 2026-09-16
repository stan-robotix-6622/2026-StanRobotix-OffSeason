// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>

#include "Configs.h"

SwerveModule::SwerveModule(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInverted, bool iTurningInverted)
{
	mDrivingMotor = new rev::spark::SparkMax{iDrivingMotorID, ModuleConstants::kDrivingMotorType};
	mTurningMotor = new rev::spark::SparkMax{iTurningMotorID, ModuleConstants::kTurningMotorType};

	// Simulation
	if (frc::RobotBase::IsSimulation()) {
		mDrivingGearBox = new frc::DCMotor{frc::DCMotor::NEO()};
		mTurningGearBox = new frc::DCMotor{frc::DCMotor::NEO550()};
		mDrivingMotorSim = new rev::spark::SparkMaxSim{mDrivingMotor, mDrivingGearBox};
		mTurningMotorSim = new rev::spark::SparkMaxSim{mTurningMotor, mTurningGearBox};
		mDrivingFlywheelSim = new frc::sim::FlywheelSim{frc::LinearSystemId::FlywheelSystem(*mDrivingGearBox, 0.025_kg_sq_m, ModuleConstants::kDrivingGearRatio), *mDrivingGearBox};
		mTurningFlywheelSim = new frc::sim::FlywheelSim{frc::LinearSystemId::FlywheelSystem(*mTurningGearBox, ChassisConstants::kModuleMOI, ModuleConstants::kTurningGearRatio), *mTurningGearBox};
	}

	mDrivingMotor->Configure(Configs::SwerveModule::DrivingConfig(iDrivingInverted),
	                         ModuleConstants::kDrivingResetMode,
	                         ModuleConstants::kDrivingPersistMode);
	mTurningMotor->Configure(Configs::SwerveModule::TurningConfig(iTurningInverted),
	                         ModuleConstants::kTurningResetMode,
	                         ModuleConstants::kTurningPersistMode);

	// Initialization of the motors' ClosedLoopController
	mDrivingClosedLoopController = new rev::spark::SparkClosedLoopController{mDrivingMotor->GetClosedLoopController()};
	mTurningClosedLoopController = new rev::spark::SparkClosedLoopController{mTurningMotor->GetClosedLoopController()};

	mDrivingEncoder = new rev::spark::SparkRelativeEncoder{mDrivingMotor->GetEncoder()};
	mTurningEncoder = new rev::spark::SparkRelativeEncoder{mTurningMotor->GetEncoder()};
	mTurningAbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{mTurningMotor->GetAbsoluteEncoder()};

	refreshModule();
	seedEncoder();
}

void SwerveModule::setDesiredState(frc::SwerveModuleState iDesiredState)
{
	mTurningCurrentAngle = frc::Rotation2d(units::radian_t(mTurningEncoder->GetPosition()));
	mOptimizedState = iDesiredState;
	mOptimizedState.Optimize(mTurningCurrentAngle);
	mOptimizedState.CosineScale(mTurningCurrentAngle);

	mDrivingClosedLoopController->SetSetpoint(mOptimizedState.speed.value(), ModuleConstants::kDrivingClosedLoopControlType);
	mTurningClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value(), ModuleConstants::kTurningClosedLoopControlType);

	if (frc::RobotBase::IsSimulation()) {
		mDrivingFlywheelSim->SetInputVoltage(units::volt_t(mDrivingMotor->GetBusVoltage()));
		mTurningFlywheelSim->SetInputVoltage(units::volt_t(mTurningMotor->GetBusVoltage()));
		mDrivingFlywheelSim->Update(0.02_s);
		mTurningFlywheelSim->Update(0.02_s);
		mDrivingMotorSim->iterate(mDrivingFlywheelSim->GetAngularVelocity().value(), 12, 0.02);
		mTurningMotorSim->iterate(mTurningFlywheelSim->GetAngularVelocity().value(), 12, 0.02);
	}
}

void SwerveModule::setDesiredHeading(frc::Rotation2d iDesiredHeading)
{
	mTurningClosedLoopController->SetSetpoint(iDesiredHeading.Radians().value(), ModuleConstants::kTurningClosedLoopControlType);

	if (frc::RobotBase::IsSimulation()) {
		mTurningMotorSim->iterate((iDesiredHeading.Radians().value() - mTurningMotorSim->GetPosition()) / 0.02, 12, 0.02);
	}
}

void SwerveModule::setDrivingVoltage(units::volt_t iVoltage)
{
	mDrivingMotor->SetVoltage(iVoltage);
}

void SwerveModule::setTurningVoltage(units::volt_t iVoltage)
{
	mTurningMotor->SetVoltage(iVoltage);
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
	return units::radians_per_second_t(mTurningEncoder->GetVelocity());
}

void SwerveModule::seedEncoder()
{
	mTurningEncoder->SetPosition(mTurningAbsoluteEncoder->GetPosition());
}

void SwerveModule::refreshModule()
{
	mModuleState = frc::SwerveModuleState{units::meters_per_second_t(mDrivingEncoder->GetVelocity()),
	                                      frc::Rotation2d(units::radian_t(mTurningEncoder->GetPosition()))};
	mModulePosition = frc::SwerveModulePosition{units::meter_t(mDrivingEncoder->GetPosition()),
	                                            frc::Rotation2d(units::radian_t(mTurningEncoder->GetPosition()))};
}

void SwerveModule::InitSendable(wpi::SendableBuilder& builder)
{
	builder.SetSmartDashboardType("swerve/module");
	builder.AddDoubleProperty("turning velocity", [this] { return mTurningEncoder->GetVelocity(); }, nullptr);
	builder.AddDoubleProperty("turning position", [this] { return mTurningEncoder->GetPosition(); }, nullptr);
	builder.AddDoubleProperty("driving velocity", [this] { return mDrivingEncoder->GetVelocity(); }, nullptr);
}
