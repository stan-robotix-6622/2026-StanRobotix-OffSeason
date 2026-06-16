#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include <string>

#include <units/angle.h>
#include <units/length.h>

namespace robotixLib
{
	// Made from the example code at https://www.chiefdelphi.com/uploads/default/original/3X/b/a/ba7ccfd90bac0934e374dd4459d813cee2903942.pdf
	double deadband(double iInput, double iThreshold, bool iSquared = false);

	namespace odometryUtils
	{
		units::degree_t GetAngleToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation);

		units::meter_t GetDistanceToTarget(frc::Translation2d iCurrentTranslation, frc::Translation2d iTargetTranslation);
	} // namespace odometryUtils

	namespace pathplannerUtils
	{
		frc::Pose2d getStartingPoseOfAuto(std::string iAutoName);
	} // namespace pathplannerUtils

	namespace Xbox
	{
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
	} // namespace Xbox
} // namespace robotixLib
