#pragma once

#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr double kJoystickDeadband = 0.05;

} // namespace OperatorConstants

namespace CANid {

inline constexpr int kFLDrive = 12;
inline constexpr int kFLSteer = 14;
inline constexpr int kFLEncoder = 32;

inline constexpr int kFRDrive = 18;
inline constexpr int kFRSteer = 16;
inline constexpr int kFREncoder = 33;

inline constexpr int kPigeon2 = 0;

} // namespace CANid

namespace CarDriveConstants {

inline constexpr double kSpeedScale = 0.5;
inline constexpr units::angle::degree_t kMaxSteerAngle{60.0_deg};
inline constexpr double kSteerGearRatio = 150.0 / 7.0;

inline constexpr units::length::inch_t kTrackWidth{14.0_in};
inline constexpr units::length::inch_t kWheelBase{21.0_in};
inline constexpr units::length::inch_t kWheelRadius{2.0_in};

inline constexpr units::current::ampere_t kSupplyCurrentLimit{40.0_A};
inline constexpr units::current::ampere_t kStatorCurrentLimit{60.0_A};

inline constexpr double kSteerP = 40.0;
inline constexpr double kSteerI = 0.0;
inline constexpr double kSteerD = 0.1;
inline constexpr double kSteerS = 0.0;

inline constexpr units::angle::turn_t kFLMagnetOffset{0.0_tr};
inline constexpr units::angle::turn_t kFRMagnetOffset{0.0_tr};

inline constexpr bool kFLDriveInverted = true;
inline constexpr bool kFRDriveInverted = true;
inline constexpr bool kFLSteerInverted = false;
inline constexpr bool kFRSteerInverted = false;

inline constexpr double kDriveGearRatio = 6.75;
inline constexpr double kTractionSlipThreshold = 2.0;
inline constexpr double kTractionKp = 0.3;
inline constexpr double kYawSlipThreshold = 20.0;
inline constexpr double kYawStabilityKp = 0.02;
inline constexpr double kDriftTorqueVectorScale = 1.25;

} // namespace CarDriveConstants
