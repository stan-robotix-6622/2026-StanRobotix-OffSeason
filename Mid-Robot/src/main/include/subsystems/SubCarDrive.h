#pragma once

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

class SubCarDrive : public frc2::SubsystemBase {
 public:
  SubCarDrive();
  ~SubCarDrive() override;

  void drive(double iThrottle, double iBrake, double iSteer, bool iReverse = false, bool iDrift = false);
  void stop();
  void stopDrive();
  void setSteerAngle(units::angle::degree_t iAngle);
  frc2::CommandPtr getTestSteerCommand();
  void zeroFL();
  void zeroFR();
  frc2::CommandPtr getZeroFLCommand();
  frc2::CommandPtr getZeroFRCommand();
  frc2::CommandPtr getCalibrateRatioCommand();

  void Periodic() override;

 private:
  void initDashboard();
  void updateConfigsFromDashboard();
  void updateTelemetry();
  void syncSteerToCANcoder();

  ctre::phoenix6::hardware::TalonFX* mFLDrive;
  ctre::phoenix6::hardware::TalonFX* mFLSteer;
  ctre::phoenix6::hardware::CANcoder* mFLEncoder;

  ctre::phoenix6::hardware::TalonFX* mFRDrive;
  ctre::phoenix6::hardware::TalonFX* mFRSteer;
  ctre::phoenix6::hardware::CANcoder* mFREncoder;

  ctre::phoenix6::hardware::Pigeon2* mPigeon;

  ctre::phoenix6::controls::DutyCycleOut mDriveDutyCycleControl{0.0};
  ctre::phoenix6::controls::PositionVoltage mSteerPositionControl{0_tr};

  double mP;
  double mI;
  double mD;
  double mS;

  units::angle::degree_t mTargetSteerAngle{0.0_deg};
  units::angle::degree_t mTargetSteerAngleFL{0.0_deg};
  units::angle::degree_t mTargetSteerAngleFR{0.0_deg};
  double mTargetSpeedFL{0.0};
  double mTargetSpeedFR{0.0};

  double mFLMagnetOffset;
  double mFRMagnetOffset;

  bool mFLDriveInverted;
  bool mFRDriveInverted;
  bool mFLSteerInverted;
  bool mFRSteerInverted;

  double mSteerGearRatio;
  units::angle::degree_t mMaxSteerAngle;
  double mSpeedScale;

  units::length::inch_t mTrackWidth;
  units::length::inch_t mWheelBase;
  units::length::inch_t mWheelRadius;

  double mDriveGearRatio;
  double mTractionSlipThreshold;
  double mTractionKp;
  double mYawSlipThreshold;
  double mYawStabilityKp;
  double mDriftTorqueVectorScale;
  bool mTractionControlEnabled{true};

  bool mIsSlipping{false};
  bool mDriftActive{false};
  int mTelemetryCounter{0};

  double mFilteredWheelAccelFL{0.0};
  double mFilteredWheelAccelFR{0.0};
  double mPrevWheelSpeedFL{0.0};
  double mPrevWheelSpeedFR{0.0};
  double mFilteredImuAx{0.0};
};
