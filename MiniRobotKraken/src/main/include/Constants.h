// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

}  // namespace OperatorConstants

namespace DriveTrainConstants {
  constexpr int kRightMotorDriveID = 18;
  constexpr int kRightMotorTurnID = 16;
  constexpr int kRightEncoder = 33;

  constexpr int kLeftMotorDriveID = 12;
  constexpr int kLeftMotorTurnID = 14;
  constexpr int kLeftEncoder = 32;
    
}

namespace PIDConstants{

	inline constexpr double kP = 0;
	inline constexpr double kI = 0;
	inline constexpr double kD = 0;

}