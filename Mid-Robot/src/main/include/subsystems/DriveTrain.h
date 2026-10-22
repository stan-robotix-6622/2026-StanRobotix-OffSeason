#pragma once

#include <string_view>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <frc/PowerDistribution.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/length.h>
#include "Constants.h"

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  ~DriveTrain() override;

  /**
   * Conduite avec gâchettes et stick (Throttle, Brake, Steer, Reverse, Drift).
   */
  void Drive(double iThrottle, double iBrake, double iSteer, bool iReverse = false, bool iDrift = false);

  /**
   * Conduite Arcade standard (Speed [-1, 1], Rotation [-1, 1], Drift).
   */
  void DriveArcade(double iSpeed, double iRotation, bool iDrift = false);

  /**
   * Commande autonome de drift à gauche.
   * @param throttle Puissance globale du drift (0.1 à 1.0)
   */
  void DriftLeft(double throttle = 1.0);

  /**
   * Commande autonome de drift à droite.
   * @param throttle Puissance globale du drift (0.1 à 1.0)
   */
  void DriftRight(double throttle = 1.0);

  /**
   * Arrêt complet de la propulsion et recentrage direction.
   */
  void Stop();

  /**
   * Règle l'angle de direction des modules BL et BR.
   */
  void SetSteerAngle(units::angle::degree_t iAngle);

  // Calibration et zérotage des modules
  void ZeroBL();
  void ZeroBR();
  frc2::CommandPtr GetZeroBLCommand();
  frc2::CommandPtr GetZeroBRCommand();

  void Periodic() override;

 private:
  void InitDashboard();
  void UpdateConfigsFromDashboard();
  void UpdateTelemetry();
  void SyncSteerToCANcoder();
  void ApplyMagnetConfig(ctre::phoenix6::hardware::CANcoder* iEncoder, units::angle::turn_t iOffset);
  void ApplyDriveConfig(ctre::phoenix6::hardware::TalonFX* iMotor, bool iInverted);
  void ApplySteerConfig(ctre::phoenix6::hardware::TalonFX* iMotor, bool iInverted);
  void ZeroSteer(ctre::phoenix6::hardware::CANcoder* iEncoder, ctre::phoenix6::hardware::TalonFX* iSteer,
                 units::angle::turn_t& ioMagnetOffset, std::string_view iKey);

  // Module Back-Left (BL)
  ctre::phoenix6::hardware::TalonFX* mBLDrive{nullptr};
  ctre::phoenix6::hardware::TalonFX* mBLSteer{nullptr};
  ctre::phoenix6::hardware::CANcoder* mBLEncoder{nullptr};

  // Module Back-Right (BR)
  ctre::phoenix6::hardware::TalonFX* mBRDrive{nullptr};
  ctre::phoenix6::hardware::TalonFX* mBRSteer{nullptr};
  ctre::phoenix6::hardware::CANcoder* mBREncoder{nullptr};

  // Capteurs & PDP
  ctre::phoenix6::hardware::Pigeon2* mPigeon{nullptr};
  frc::PowerDistribution mPDP{CANid::kPDP, frc::PowerDistribution::ModuleType::kCTRE};

  ctre::phoenix6::controls::DutyCycleOut mDriveDutyCycleControl{0.0};
  ctre::phoenix6::controls::PositionVoltage mSteerPositionControl{0_tr};

  double mSteerP{DriveConstants::kSteerP};
  double mSteerI{DriveConstants::kSteerI};
  double mSteerD{DriveConstants::kSteerD};
  double mSteerS{DriveConstants::kSteerS};

  units::angle::degree_t mTargetSteerAngle{0.0_deg};
  units::angle::degree_t mTargetSteerAngleBL{0.0_deg};
  units::angle::degree_t mTargetSteerAngleBR{0.0_deg};
  double mTargetSpeedBL{0.0};
  double mTargetSpeedBR{0.0};

  units::angle::turn_t mBLMagnetOffset{DriveConstants::kBLMagnetOffset};
  units::angle::turn_t mBRMagnetOffset{DriveConstants::kBRMagnetOffset};

  bool mBLDriveInverted{DriveConstants::kBLDriveInverted};
  bool mBRDriveInverted{DriveConstants::kBRDriveInverted};
  bool mBLSteerInverted{DriveConstants::kBLSteerInverted};
  bool mBRSteerInverted{DriveConstants::kBRSteerInverted};

  double mSteerGearRatio{DriveConstants::kSteerGearRatio};
  units::angle::degree_t mMaxSteerAngle{DriveConstants::kMaxSteerAngle};
  double mSpeedScale{DriveConstants::kSpeedScale};

  units::meter_t mTrackWidth{DriveConstants::kTrackWidth};
  units::meter_t mWheelBase{DriveConstants::kWheelBase};

  double mDriftTorqueVectorScale{DriveConstants::kDriftTorqueVectorScale};
  double mDriftInnerScale{DriveConstants::kDriftInnerScale};

  bool mDriftActive{false};
  int mTelemetryCounter{0};
};
