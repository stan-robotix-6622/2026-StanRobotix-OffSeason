#pragma once

#include <units/angle.h>
#include <units/current.h>

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr double kJoystickDeadband = 0.05;

} // namespace OperatorConstants

namespace CANid {

inline constexpr int kFLDrive = 15;
inline constexpr int kFLSteer = 14;
inline constexpr int kFLEncoder = 32;

inline constexpr int kFRDrive = 17;
inline constexpr int kFRSteer = 16;
inline constexpr int kFREncoder = 33;

} // namespace CANid

namespace CarDriveConstants {

inline constexpr double kSpeedScale = 0.5;
inline constexpr units::angle::degree_t kMaxSteerAngle{60.0};
inline constexpr double kSteerGearRatio = 150.0 / 7.0;

inline constexpr units::current::ampere_t kSupplyCurrentLimit{40.0};
inline constexpr units::current::ampere_t kStatorCurrentLimit{60.0};

inline constexpr double kSteerP = 40.0;
inline constexpr double kSteerI = 0.0;
inline constexpr double kSteerD = 0.1;
inline constexpr double kSteerS = 0.0;

inline constexpr units::angle::turn_t kFLMagnetOffset{0.0};
inline constexpr units::angle::turn_t kFRMagnetOffset{0.0};

inline constexpr bool kFLDriveInverted = true;
inline constexpr bool kFRDriveInverted = true;
inline constexpr bool kFLSteerInverted = false;
inline constexpr bool kFRSteerInverted = false;

} // namespace CarDriveConstants
