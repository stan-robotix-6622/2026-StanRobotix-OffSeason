// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.hpp"

#include <wpi/framework/RobotBase.hpp>
// #include <wpi/smartdashboard/SmartDashboard.hpp>
#include <wpi/math/system/Models.hpp>

#include "Configs.hpp"

SwerveModule::SwerveModule(int iDrivingMotorID, int iTurningMotorID, bool iDrivingInverted, bool iTurningInverted)
{
	mDrivingMotor = new rev::spark::SparkMax{wpi::CANPort::CAN_D0, iDrivingMotorID, ModuleConstants::kDrivingMotorType};
	mTurningMotor = new rev::spark::SparkMax{wpi::CANPort::CAN_D0, iTurningMotorID, ModuleConstants::kTurningMotorType};

	// Simulation
	if (wpi::RobotBase::IsSimulation()) {
		mDrivingGearBox = new wpi::math::DCMotor{wpi::math::DCMotor::NEO()};
		mTurningGearBox = new wpi::math::DCMotor{wpi::math::DCMotor::NEO550()};
		mDrivingMotorSim = new rev::spark::SparkMaxSim{mDrivingMotor, mDrivingGearBox};
		mTurningMotorSim = new rev::spark::SparkMaxSim{mTurningMotor, mTurningGearBox};
		mDrivingFlywheelSim = new wpi::sim::FlywheelSim{wpi::math::Models::FlywheelFromPhysicalConstants(*mDrivingGearBox, 0.025_kg_sq_m, ModuleConstants::kDrivingGearRatio), *mDrivingGearBox};
		mTurningFlywheelSim = new wpi::sim::FlywheelSim{wpi::math::Models::FlywheelFromPhysicalConstants(*mTurningGearBox, ChassisConstants::kModuleMOI, ModuleConstants::kTurningGearRatio), *mTurningGearBox};
	}

	mDrivingMotor->Configure(Configs::SwerveModule::DrivingConfig(iDrivingInverted),
	                         ModuleConstants::kDrivingResetMode,
	                         ModuleConstants::kDrivingPersistMode);
	mTurningMotor->Configure(Configs::SwerveModule::TurningConfig(iTurningInverted),
	                         ModuleConstants::kTurningResetMode,
	                         ModuleConstants::kTurningPersistMode);

	mDrivingFeedforward = new wpi::math::SimpleMotorFeedforward<wpi::units::meters>{0_V, 12_V / ModuleConstants::kDriveWheelMaxFreeSpeed};
	mTurningFeedforward = new wpi::math::SimpleMotorFeedforward<wpi::units::radians>{0_V, 12_V / ModuleConstants::kTurningWheelFreeSpeedRadps};
	mDrivingPID = new wpi::math::PIDController{ModuleConstants::kDrivingP, ModuleConstants::kDrivingI, ModuleConstants::kDrivingD};
	mTurningPID = new wpi::math::PIDController{ModuleConstants::kTurningP, ModuleConstants::kTurningI, ModuleConstants::kTurningD};
	mTurningPID->EnableContinuousInput(ModuleConstants::Config::kTurningClosedLoopMinInput, ModuleConstants::Config::kTurningClosedLoopMaxInput);

	// Initialization of the motors' ClosedLoopController
	mDrivingClosedLoopController = new rev::spark::SparkClosedLoopController{mDrivingMotor->GetClosedLoopController()};
	mTurningClosedLoopController = new rev::spark::SparkClosedLoopController{mTurningMotor->GetClosedLoopController()};

	mDrivingEncoder = new rev::spark::SparkRelativeEncoder{mDrivingMotor->GetEncoder()};
	mTurningEncoder = new rev::spark::SparkRelativeEncoder{mTurningMotor->GetEncoder()};
	mTurningAbsoluteEncoder = new rev::spark::SparkAbsoluteEncoder{mTurningMotor->GetAbsoluteEncoder()};

	refreshModule();
	seedEncoder();
}

void SwerveModule::setDesiredState(wpi::math::SwerveModuleVelocity iDesiredState)
{
	mTurningCurrentAngle = wpi::math::Rotation2d(wpi::units::radian_t(mTurningEncoder->GetPosition().Get()));
	mOptimizedState = iDesiredState;
	mOptimizedState = mOptimizedState.Optimize(mTurningCurrentAngle);
	mOptimizedState = mOptimizedState.CosineScale(mTurningCurrentAngle);

	mDrivingClosedLoopController->SetSetpoint(mOptimizedState.velocity.value(), ModuleConstants::kDrivingClosedLoopControlType);
	mTurningClosedLoopController->SetSetpoint(mOptimizedState.angle.Radians().value(), ModuleConstants::kTurningClosedLoopControlType);

	if (wpi::RobotBase::IsSimulation()) {
		mDrivingPID->SetSetpoint(mOptimizedState.velocity.value());
		mTurningPID->SetSetpoint(mOptimizedState.angle.Radians().value());
		// wpi::SmartDashboard::PutNumber("drivetrain/swerve turning pid output", mTurningPID->Calculate(mTurningMotorSim->GetVelocity()));
		// mDrivingFlywheelSim->SetInputVoltage(wpi::units::volt_t(
		//   mDrivingFeedforward->Calculate(wpi::units::meters_per_second_t(mTurningMotorSim->GetVelocity()), wpi::units::meters_per_second_t(mDrivingPID->Calculate(mDrivingMotorSim->GetVelocity())))
		// ));
		mTurningFlywheelSim->SetInputVoltage(wpi::units::volt_t(
				mTurningFeedforward->Calculate(wpi::units::radians_per_second_t(mDrivingMotorSim->GetVelocity()), wpi::units::radians_per_second_t(mTurningPID->Calculate(mTurningMotorSim->GetVelocity())))
		));
		// wpi::SmartDashboard::PutNumber("drivetrain/swerve module turning voltage", mTurningFeedforward->Calculate(wpi::units::radians_per_second_t(mDrivingMotorSim->GetVelocity()), wpi::units::radians_per_second_t(mTurningPID->Calculate(mTurningMotorSim->GetVelocity()))).value());
		// mDrivingFlywheelSim->Update(0.02_s);
		mTurningFlywheelSim->Update(0.02_s);
		// mDrivingMotorSim->iterate(mDrivingFlywheelSim->GetAngularVelocity().value(), 12, 0.02);
		mTurningMotorSim->iterate(mTurningFlywheelSim->GetAngularVelocity().value(), 12, 0.02);
	}
}

void SwerveModule::setDesiredHeading(wpi::math::Rotation2d iDesiredHeading)
{
	mTurningClosedLoopController->SetSetpoint(iDesiredHeading.Radians().value(), ModuleConstants::kTurningClosedLoopControlType);

	if (wpi::RobotBase::IsSimulation()) {
		mTurningMotorSim->iterate((iDesiredHeading.Radians().value() - mTurningMotorSim->GetPosition()) / 0.02, 12, 0.02);
	}
}

void SwerveModule::setDrivingVoltage(wpi::units::volt_t iVoltage)
{
	mDrivingMotor->SetVoltage(iVoltage);
}

void SwerveModule::setTurningVoltage(wpi::units::volt_t iVoltage)
{
	mTurningMotor->SetVoltage(iVoltage);
}

wpi::math::SwerveModuleVelocity SwerveModule::getModuleVelocity()
{
	return mModuleVelocity;
}

wpi::math::SwerveModulePosition SwerveModule::getModulePosition()
{
	return mModulePosition;
}

wpi::units::radians_per_second_t SwerveModule::getTurningVelocity()
{
	return wpi::units::radians_per_second_t(mTurningEncoder->GetVelocity().Get());
}

void SwerveModule::seedEncoder()
{
	mTurningEncoder->SetPosition(mTurningAbsoluteEncoder->GetPosition().Get());
}

void SwerveModule::refreshModule()
{
	mModuleVelocity = wpi::math::SwerveModuleVelocity{wpi::units::meters_per_second_t(mDrivingEncoder->GetVelocity().Get()),
	                                      wpi::math::Rotation2d(wpi::units::radian_t(mTurningEncoder->GetPosition().Get()))};
	mModulePosition = wpi::math::SwerveModulePosition{wpi::units::meter_t(mDrivingEncoder->GetPosition().Get()),
	                                            wpi::math::Rotation2d(wpi::units::radian_t(mTurningEncoder->GetPosition().Get()))};
}

// void SwerveModule::InitSendable(wpi::util::SendableBuilder& builder)
// {
// 	builder.SetSmartDashboardType("swerve/module");
// 	builder.AddDoubleProperty("turning velocity", [this] { return mTurningEncoder->GetVelocity(); }, nullptr);
// 	builder.AddDoubleProperty("turning position", [this] { return mTurningEncoder->GetPosition(); }, nullptr);
// 	builder.AddDoubleProperty("driving velocity", [this] { return mDrivingEncoder->GetVelocity(); }, nullptr);
// }
