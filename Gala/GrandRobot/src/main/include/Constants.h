// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Translation2d.h>

#include <numbers>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace TemplateUnits
{
	template <typename Unit>
	using VoltageInverse = units::unit_t<units::detail::unit_multiply<units::voltage::volts, units::inverse<Unit>>, double, units::linear_scale>;
} // namespace TemplateUnits

namespace OperatorConstants
{
	inline constexpr int kDriverControllerPort = 0;
	inline constexpr int kCopilotControllerPort = 1;

	namespace Button
	{
		inline constexpr int A = 1;
		inline constexpr int B = 2;
		inline constexpr int X = 3;
		inline constexpr int Y = 4;
		inline constexpr int LeftBumper = 5;
		inline constexpr int RightBumper = 6;
		inline constexpr int Back = 7;
		inline constexpr int Start = 8;
		inline constexpr int LeftJoystick = 9;
		inline constexpr int RightJoystick = 10;
	} // namespace Button

	namespace Axis
	{
		inline constexpr int LeftX = 0;
		inline constexpr int LeftY = 1;
		inline constexpr int LeftTrigger = 2;
		inline constexpr int RightTrigger = 3;
		inline constexpr int RightX = 4;
		inline constexpr int RightY = 5;
	} // namespace Axis
} // namespace OperatorConstants

namespace PathPlannerConstants
{
	inline constexpr double kPTranslation = 5.0;
	inline constexpr double kITranslation = 0.0;
	inline constexpr double kDTranslation = 0.0;
	inline constexpr double kPRotation = 5.0;
	inline constexpr double kIRotation = 0.0;
	inline constexpr double kDRotation = 0.0;

	inline constexpr units::meters_per_second_t kMaxVelocity = .3_mps;
	inline constexpr units::meters_per_second_squared_t kMaxAcceleration = 0.3_mps_sq;
	inline constexpr units::degrees_per_second_t kMaxAngularVelocity = 36.0_deg_per_s;
	inline constexpr units::degrees_per_second_squared_t kMaxAngularAcceleration = 72.0_deg_per_s_sq;
} // namespace PathPlannerConstants

namespace ModuleConstants
{
	inline constexpr double kDrivingMotorGearRatio = 4.71;                                 // 4.71 rotations of the motor for 1 rotation of the ouput
	inline constexpr double kTurningGearRatio = 9424 / 203;                                // 9424 rotations of the motor for 203 rotations of the output
	inline constexpr units::volt_t kNominalVoltage = 12_V;                                 // The voltage at which the max speeds are mesured
	inline constexpr units::meter_t kWheelRadius = 1.341628_in;                            // The radius of REV's plastic wheels, masured with the wheelCaracterizationCommand
	inline constexpr units::meter_t kWheelPerimeter = kWheelRadius * 2 * std::numbers::pi; // in meters (diametre in inches * convertion to meters * pi)
	inline constexpr units::radians_per_second_t kTurningWheelFreeSpeedRadps = 24.260_rad_per_s;
	inline constexpr units::meters_per_second_t kDriveWheelMaxFreeSpeed = 4.9180_mps;

	inline constexpr double kDrivingFactor = ModuleConstants::kWheelPerimeter.value() / kDrivingMotorGearRatio;
	inline constexpr double kTurningFactor = 2 * std::numbers::pi;

	// inline constexpr rev::spark::SparkLowLevel::ControlType kDrivingClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kVelocity;
	// inline constexpr rev::spark::SparkLowLevel::ControlType kTurningClosedLoopControlType = rev::spark::SparkLowLevel::ControlType::kPosition;

	// inline constexpr rev::spark::SparkLowLevel::MotorType kDrivingMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;
	// inline constexpr rev::spark::SparkLowLevel::MotorType kTurningMotorType = rev::spark::SparkLowLevel::MotorType::kBrushless;

	inline constexpr units::ampere_t kDrivingCurrentLimit = 80_A;
	inline constexpr units::ampere_t kTurningCurrentLimit = 20_A;

	// inline constexpr rev::ResetMode kDrivingResetMode = rev::ResetMode::kResetSafeParameters;
	// inline constexpr rev::ResetMode kTurningResetMode = rev::ResetMode::kResetSafeParameters;

	// inline constexpr rev::PersistMode kDrivingPersistMode = rev::PersistMode::kPersistParameters;
	// inline constexpr rev::PersistMode kTurningPersistMode = rev::PersistMode::kPersistParameters;

	inline constexpr double kTurningP = 0.3;
	inline constexpr double kTurningI = 0.0;
	inline constexpr double kTurningD = 0.0;
	inline constexpr double kDrivingP = 0.04;
	inline constexpr double kDrivingI = 0.0;
	inline constexpr double kDrivingD = 0.0;

	namespace Config
	{
		inline constexpr double kRPMtoRPSFactor = 60;

		inline constexpr units::revolutions_per_minute_t kTurningCruiseVelocity = 2_rad_per_s * std::numbers::pi;
		inline constexpr units::revolutions_per_minute_per_second_t kTurningMaxAcceleration = 4_rad_per_s_sq * std::numbers::pi;

		// inline constexpr rev::spark::SparkBaseConfig::IdleMode kDrivingIdleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
		// inline constexpr rev::spark::SparkBaseConfig::IdleMode kTurningIdleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;

		// inline constexpr rev::spark::FeedbackSensor kDrivingClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kPrimaryEncoder;
		// inline constexpr rev::spark::FeedbackSensor kTurningClosedLoopFeedbackSensor = rev::spark::FeedbackSensor::kPrimaryEncoder;

		inline constexpr bool kTurningMotorInverted = false;
		inline constexpr bool kTurningEncoderZeroCentered = true;
		inline constexpr bool kTurningClosedLoopPositionWrapping = true;
		inline constexpr double kTurningClosedLoopMinInput = -std::numbers::pi;
		inline constexpr double kTurningClosedLoopMaxInput = std::numbers::pi;
		inline constexpr double kTurningClosedLoopTolerance = std::numbers::pi / 360;
	} // namespace Config
} // namespace ModuleConstants

namespace DrivetrainConstants
{
	// Left-Right
	inline constexpr units::meter_t kRobotWidth = 28_in;
	// Front-Back
	inline constexpr units::meter_t kRobotLength = 26.875_in;
	// In both directions
	inline constexpr units::meter_t kModuleCornerOffset = 1.75_in;

	// We take for granted a rectangular frame
	inline constexpr frc::Translation2d kFrontLeftTranslation = frc::Translation2d{(kRobotLength / 2 - kModuleCornerOffset), (kRobotWidth / 2 - kModuleCornerOffset)};
	inline constexpr frc::Translation2d kFrontRightTranslation = frc::Translation2d{(kRobotLength / 2 - kModuleCornerOffset), -(kRobotWidth / 2 - kModuleCornerOffset)};
	inline constexpr frc::Translation2d kBackLeftTranslation = frc::Translation2d{-(kRobotLength / 2 - kModuleCornerOffset), (kRobotWidth / 2 - kModuleCornerOffset)};
	inline constexpr frc::Translation2d kBackRightTranslation = frc::Translation2d{-(kRobotLength / 2 - kModuleCornerOffset), -(kRobotWidth / 2 - kModuleCornerOffset)};

	inline constexpr units::meters_per_second_t kAttainableSpeed = 4.50_mps;
	inline constexpr units::meters_per_second_t kMaxDesiredSpeed = 4.50_mps;
	inline constexpr units::radians_per_second_t kMaxDesiredAngularSpeed = std::numbers::pi * 3_rad_per_s;

	namespace Commands
	{
		inline constexpr units::second_t kMaxSpeedStartDelay = 2.0_s;
		inline constexpr units::meters_per_second_squared_t kMaxSpeedRampRate = 0.5_mps_sq;
		inline constexpr units::meters_per_second_t kMaxSpeedMaxVelocity = 5_mps;
		inline constexpr units::second_t kFeedforwartStartDelay = 2.0_s;
		inline constexpr TemplateUnits::VoltageInverse<units::seconds> kFeedforwardRampRate = 1_V / 1_s;
		inline constexpr units::second_t kWheelRadiusMeasurementStartDelay = 1.0_s;
		inline constexpr units::radians_per_second_t kWheelRadiusMaxVelocity = 0.25_rad_per_s;
		inline constexpr units::radians_per_second_squared_t kWheelRadiusRampRate = 0.05_rad_per_s_sq;
	} // namespace Commands
} // namespace DrivetrainConstants

namespace LimelightConstants
{
	inline constexpr bool kUseMegaTag2 = true;

	inline constexpr std::string_view kName = "limelight";

	inline constexpr units::meter_t kForward = 13.6875_in;
	inline constexpr units::meter_t kRight = -11.875_in;
	inline constexpr units::meter_t kUp = 20.875_in;

	inline constexpr units::degree_t kRoll = 0_deg;
	inline constexpr units::degree_t kPitch = 0_deg;
	inline constexpr units::degree_t kYaw = 0_deg;

	inline constexpr double kPoseEstimatorStandardDeviationX = 0.7;      // Default/Recommended values
	inline constexpr double kPoseEstimatorStandardDeviationY = 0.7;      // Default/Recommended values
	inline constexpr double kPoseEstimatorStandardDeviationYaw = 999999; // Default/Recommended values
} // namespace LimelightConstants

namespace CANid
{
	inline constexpr int kBackRightMotorID = 8;
	inline constexpr int kBackRightMotor550ID = 7;
	inline constexpr int kFrontRightMotorID = 4;
	inline constexpr int kFrontRightMotor550ID = 3;
	inline constexpr int kFrontLeftMotorID = 6;
	inline constexpr int kFrontLeftMotor550ID = 5;
	inline constexpr int kBackLeftMotorID = 2;
	inline constexpr int kBackLeftMotor550ID = 1;
  
	inline constexpr int kIMUPigeonID = 0;
} // namespace CANid
