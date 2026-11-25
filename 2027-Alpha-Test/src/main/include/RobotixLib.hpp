#pragma once

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>

#include <string>

#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/voltage.hpp>

namespace robotixLib
{
	// Made from the example code at https://www.chiefdelphi.com/uploads/default/original/3X/b/a/ba7ccfd90bac0934e374dd4459d813cee2903942.pdf
	double deadband(double iInput, double iThreshold, bool iSquared = false);

	namespace odometryUtils
	{
		wpi::units::degree_t GetAngleToTarget(wpi::math::Translation2d iCurrentTranslation, wpi::math::Translation2d iTargetTranslation);

		wpi::units::meter_t GetDistanceToTarget(wpi::math::Translation2d iCurrentTranslation, wpi::math::Translation2d iTargetTranslation);
	} // namespace odometryUtils
	
	namespace pathplannerUtils
	{
		wpi::math::Pose2d getStartingPoseOfAuto(std::string iAutoName);
	} // namespace pathplannerUtils

	namespace templateUnits
	{
		template <typename Unit>
		using VoltageInverse = wpi::units::unit_t<wpi::units::compound_unit<wpi::units::volts, wpi::units::inverse<Unit>>>;
	} // namespace templateUnits
} // namespace robotixLib
