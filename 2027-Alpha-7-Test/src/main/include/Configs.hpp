#pragma once

#include <rev/config/SparkMaxConfig.h>

#include "Constants.hpp"

namespace Configs
{
	using namespace rev::spark;
	class SwerveModule {
	 public:
		static SparkMaxConfig& DrivingConfig(bool iDrivingInverted)
		{
			static SparkMaxConfig drivingConfig{};

			// constexpr double drivingFactor = ModuleConstants::kDrivingFactor;
			constexpr robotixLib::templateUnits::VoltageInverse<wpi::units::meters_per_second> drivingVelocityFeedForward = ModuleConstants::kNominalVoltage / ModuleConstants::kDriveWheelMaxFreeSpeed;

			drivingConfig.Inverted(iDrivingInverted);
			drivingConfig.SetIdleMode(ModuleConstants::Config::kDrivingIdleMode);

			// drivingConfig.encoder.VelocityConversionFactor(drivingFactor / ModuleConstants::Config::kRPMtoRPSFactor);
			// drivingConfig.encoder.PositionConversionFactor(drivingFactor);

			drivingConfig.closedLoop.SetFeedbackSensor(ModuleConstants::Config::kDrivingClosedLoopFeedbackSensor);
			drivingConfig.closedLoop.Pid(ModuleConstants::kDrivingP, ModuleConstants::kDrivingI, ModuleConstants::kDrivingD);
			drivingConfig.closedLoop.OutputRange(-1, 1);

			drivingConfig.closedLoop.feedForward.kV(drivingVelocityFeedForward.value());

			drivingConfig.SmartCurrentLimit(ModuleConstants::kDrivingCurrentLimit.value());

			return drivingConfig;
		}

		static SparkMaxConfig& TurningConfig(bool iEncoderInverted)
		{
			static SparkMaxConfig turningConfig{};

			// constexpr double turningAbsoluteFactor = ModuleConstants::kTurningFactor;
			// constexpr double turningRelativeFactor = ModuleConstants::kTurningFactor / ModuleConstants::kTurningGearRatio;

			turningConfig.Inverted(ModuleConstants::Config::kTurningMotorInverted);
			turningConfig.SetIdleMode(ModuleConstants::Config::kTurningIdleMode);

			// turningConfig.encoder.VelocityConversionFactor(turningRelativeFactor / ModuleConstants::Config::kRPMtoRPSFactor);
			// turningConfig.encoder.PositionConversionFactor(turningRelativeFactor);

			// turningConfig.absoluteEncoder.VelocityConversionFactor(turningAbsoluteFactor / ModuleConstants::Config::kRPMtoRPSFactor);
			// turningConfig.absoluteEncoder.PositionConversionFactor(turningAbsoluteFactor);
			turningConfig.absoluteEncoder.Inverted(iEncoderInverted);
			turningConfig.absoluteEncoder.ZeroOffset(0.5);
			turningConfig.absoluteEncoder.Apply(AbsoluteEncoderConfig::Presets::REV_ThroughBoreEncoder());

			turningConfig.closedLoop.SetFeedbackSensor(ModuleConstants::Config::kTurningClosedLoopFeedbackSensor);
			turningConfig.closedLoop.Pid(ModuleConstants::kTurningP, ModuleConstants::kTurningI, ModuleConstants::kTurningD);
			turningConfig.closedLoop.OutputRange(-1, 1);
			turningConfig.closedLoop.PositionWrappingEnabled(ModuleConstants::Config::kTurningClosedLoopPositionWrapping);
			turningConfig.closedLoop.MinOutput(ModuleConstants::Config::kTurningClosedLoopMinInput);
			turningConfig.closedLoop.MaxOutput(ModuleConstants::Config::kTurningClosedLoopMaxInput);
			turningConfig.closedLoop.AllowedClosedLoopError(ModuleConstants::Config::kTurningClosedLoopTolerance);

			turningConfig.SmartCurrentLimit(ModuleConstants::kTurningCurrentLimit.value());

			return turningConfig;
		}
	};
} // namespace Configs
