// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc/util/Color.h>

#include <units/length.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr int kDriverJoystickPort = 1;

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


}  // namespace OperatorConstants


namespace PIDConstants
{
  constexpr double kP = 0.001;
  constexpr double kI = 0;
  constexpr double kD = 0;
}

namespace DriveConstants
{
<<<<<<< HEAD

  constexpr double kSpeed = 0.50;
  constexpr double kRotationRate = 0.55;
  constexpr double kSmooth = 0.2;
	constexpr double kSpeedMin = 0.5;
=======
  constexpr double kSpeed = 0.55;
  constexpr double kRotationRate = 0.5;
  constexpr double kSmooth = 0.09;
  //constexpr units::second_t kSpeedRampRate = 3_s;
>>>>>>> fc0c6ed111b17d4e621f6d159f8195a26afc2d1d

}

namespace LEDsConstants
{
	static constexpr int kLength = 20;
	constexpr units::meter_t kLedSpacing = 0.03_m;
	constexpr int kLEDPort = 9;
}

namespace CanIDConstants
{
  constexpr int kLeftCanID = 14; // to determine
  constexpr int kRightCanID = 12; // to determine
}


namespace LEDsConstants
{
	static constexpr int kLength = 100;
	constexpr units::meter_t kLedSpacing = 0.03_m;
	constexpr int kLEDPort = 9;
}