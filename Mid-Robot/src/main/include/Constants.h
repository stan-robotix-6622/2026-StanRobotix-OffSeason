#pragma once

#include <units/angle.h>
#include <units/current.h>

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr double kJoystickDeadband = 0.05;

}  // namespace OperatorConstants

namespace CANid {

inline constexpr int kFrontLeftDrive = 1;
inline constexpr int kFrontLeftSteer = 2;
inline constexpr int kFrontLeftCANcoder = 3;

inline constexpr int kFrontRightDrive = 4;
inline constexpr int kFrontRightSteer = 5;
inline constexpr int kFrontRightCANcoder = 6;

}  // namespace CANid

namespace CarDriveConstants {

inline constexpr double kSpeedScale = 0.5;
inline constexpr units::angle::degree_t kMaxSteerAngle{60.0};
inline constexpr double kSteerGearRatio = 150.0 / 7.0;

inline constexpr units::current::ampere_t kSupplyCurrentLimit{40.0};
inline constexpr units::current::ampere_t kStatorCurrentLimit{60.0};

inline constexpr double kSteerP = 24.0;
inline constexpr double kSteerI = 0.0;
inline constexpr double kSteerD = 0.2;

inline constexpr units::angle::turn_t kFrontLeftMagnetOffset{0.0};
inline constexpr units::angle::turn_t kFrontRightMagnetOffset{0.0};

inline constexpr bool kFrontLeftDriveInverted = false;
inline constexpr bool kFrontRightDriveInverted = true;
inline constexpr bool kFrontLeftSteerInverted = false;
inline constexpr bool kFrontRightSteerInverted = false;

}  // namespace CarDriveConstants
