#pragma once

#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr double kJoystickDeadband = 0.05;

}  // namespace OperatorConstants

namespace CANid {

// Back-Left (BL) Module
inline constexpr int kBLDrive = 12;    // Kraken X60 Propulsion Gauche
inline constexpr int kBLSteer = 14;    // Moteur Direction Gauche
inline constexpr int kBLEncoder = 32;  // CANcoder Direction Gauche

// Back-Right (BR) Module
inline constexpr int kBRDrive = 18;    // Kraken X60 Propulsion Droite
inline constexpr int kBRSteer = 16;    // Moteur Direction Droite
inline constexpr int kBREncoder = 33;  // CANcoder Direction Droite

// Capteurs & Électronique
inline constexpr int kPigeon2 = 0;     // Pigeon 2 IMU
inline constexpr int kPDP = 0;         // CTRE Power Distribution Panel

}  // namespace CANid

namespace DriveConstants {

// Dimensions du robot
inline constexpr units::length::inch_t kTrackWidth{14.0_in};
inline constexpr units::length::inch_t kWheelBase{21.0_in};

// Échelle et braquage
inline constexpr double kSpeedScale = 0.85;
inline constexpr units::angle::degree_t kMaxSteerAngle{60.0_deg};
inline constexpr double kSteerGearRatio = 150.0 / 7.0;

// Limites de courant Kraken X60 et Steer (Protection PDP / Batterie)
inline constexpr units::current::ampere_t kSupplyCurrentLimit{40.0_A};
inline constexpr units::current::ampere_t kStatorCurrentLimit{80.0_A};

// PID Direction (Steer)
inline constexpr double kSteerP = 40.0;
inline constexpr double kSteerI = 0.0;
inline constexpr double kSteerD = 0.1;
inline constexpr double kSteerS = 0.0;

// Offsets absolus des aimants CANcoder
inline constexpr units::angle::turn_t kBLMagnetOffset{0.0_tr};
inline constexpr units::angle::turn_t kBRMagnetOffset{0.0_tr};

// Inversions
inline constexpr bool kBLDriveInverted = true;
inline constexpr bool kBRDriveInverted = true;
inline constexpr bool kBLSteerInverted = false;
inline constexpr bool kBRSteerInverted = false;

// Paramètres de Drift (Survirage & vecteur de couple)
inline constexpr double kDriftTorqueVectorScale = 1.35; // Boost de la roue extérieure
inline constexpr double kDriftInnerScale = 0.30;       // Réduction de la roue intérieure
inline constexpr units::angle::degree_t kDriftSteerAngle{45.0_deg}; // Angle de braquage lors d'un drift déclenché par commande

}  // namespace DriveConstants
