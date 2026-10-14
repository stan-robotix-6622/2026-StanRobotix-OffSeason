#pragma once

#include <string_view>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include "Constants.h"

class SubCarDrive : public frc2::SubsystemBase {
 public:
  SubCarDrive();
  ~SubCarDrive() override;

  void drive(double iThrottle, double iBrake, double iSteer, bool iReverse = false, bool iDrift = false);
  void stop();
  void setSteerAngle(units::angle::degree_t iAngle);
  frc2::CommandPtr getTestSteerCommand();
  void zeroFL();
  void zeroFR();
  frc2::CommandPtr getZeroFLCommand();
  frc2::CommandPtr getZeroFRCommand();

  void Periodic() override;

 private:
  void initDashboard();
  void updateConfigsFromDashboard();
  void updateTelemetry();
  void syncSteerToCANcoder();
  void applyMagnetConfig(ctre::phoenix6::hardware::CANcoder* iEncoder, units::angle::turn_t iOffset);
  void applyDriveConfig(ctre::phoenix6::hardware::TalonFX* iMotor, bool iInverted);
  void applySteerConfig(ctre::phoenix6::hardware::TalonFX* iMotor, bool iInverted);
  void zeroSteer(ctre::phoenix6::hardware::CANcoder* iEncoder, ctre::phoenix6::hardware::TalonFX* iSteer,
                 units::angle::turn_t& ioMagnetOffset, std::string_view iKey);

  ctre::phoenix6::hardware::TalonFX* mFLDrive;
  ctre::phoenix6::hardware::TalonFX* mFLSteer;
  ctre::phoenix6::hardware::CANcoder* mFLEncoder;

  ctre::phoenix6::hardware::TalonFX* mFRDrive;
  ctre::phoenix6::hardware::TalonFX* mFRSteer;
  ctre::phoenix6::hardware::CANcoder* mFREncoder;

  ctre::phoenix6::hardware::Pigeon2* mPigeon;

  ctre::phoenix6::controls::DutyCycleOut mDriveDutyCycleControl{0.0};
  ctre::phoenix6::controls::PositionVoltage mSteerPositionControl{0_tr};

  double mSteerP{CarDriveConstants::kSteerP};
  double mSteerI{CarDriveConstants::kSteerI};
  double mSteerD{CarDriveConstants::kSteerD};
  double mSteerS{CarDriveConstants::kSteerS};

  units::angle::degree_t mTargetSteerAngle{0.0_deg};
  units::angle::degree_t mTargetSteerAngleFL{0.0_deg};
  units::angle::degree_t mTargetSteerAngleFR{0.0_deg};
  double mTargetSpeedFL{0.0};
  double mTargetSpeedFR{0.0};

  units::angle::turn_t mFLMagnetOffset{CarDriveConstants::kFLMagnetOffset};
  units::angle::turn_t mFRMagnetOffset{CarDriveConstants::kFRMagnetOffset};

  bool mFLDriveInverted{CarDriveConstants::kFLDriveInverted};
  bool mFRDriveInverted{CarDriveConstants::kFRDriveInverted};
  bool mFLSteerInverted{CarDriveConstants::kFLSteerInverted};
  bool mFRSteerInverted{CarDriveConstants::kFRSteerInverted};

  double mSteerGearRatio{CarDriveConstants::kSteerGearRatio};
  units::angle::degree_t mMaxSteerAngle{CarDriveConstants::kMaxSteerAngle};
  double mSpeedScale{CarDriveConstants::kSpeedScale};

  units::meter_t mTrackWidth{CarDriveConstants::kTrackWidth};
  units::meter_t mWheelBase{CarDriveConstants::kWheelBase};

  double mDriftTorqueVectorScale{CarDriveConstants::kDriftTorqueVectorScale};

  bool mDriftActive{false};
  int mTelemetryCounter{0};
};
