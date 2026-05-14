// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/util/Color.h>

#include <units/length.h>

namespace LEDsConstants
{
	static constexpr int kLength = 40;
	constexpr units::meter_t kLedSpacing = 0.03_m;
	constexpr int kLEDPort = 9;
} // namespace LEDsConstants

namespace OperatorConstants
{
	inline constexpr int kDriverControllerPort = 0;
} // namespace OperatorConstants
