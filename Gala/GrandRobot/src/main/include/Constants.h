// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/util/Color.h>

#include <units/length.h>
#include <units/velocity.h>

namespace OperatorConstants
{
	inline constexpr int kDriverControllerPort = 0;
} // namespace OperatorConstants

namespace LEDsConstants
{
	inline constexpr int kLength = 150;
	inline constexpr units::meter_t kLedSpacing = 17_mm;
	inline constexpr units::meters_per_second_t kScrollingSpeed = 1_mps;
	inline constexpr int kLEDPort = 9;
} // namespace LEDsConstants
