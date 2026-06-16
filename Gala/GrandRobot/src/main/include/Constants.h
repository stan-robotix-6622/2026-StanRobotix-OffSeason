// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc/util/Color.h>

#include <units/length.h>

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}

namespace LEDsConstants
{
	static constexpr int kLength = 130;
	constexpr units::meter_t kLedSpacing = 0.03_m;
	constexpr int kLEDPort = 9;
}