#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>

class SubCarDrive : public frc2::SubsystemBase {
 public:
  SubCarDrive();
  ~SubCarDrive() override;

  void drive(double iThrottle, double iBrake, double iSteer);
  void stop();
  void stopDrive();
  void setSteerAngle(units::angle::degree_t iAngle);
  frc2::CommandPtr getTestSteerCommand();

  void Periodic() override;

 private:
  void initDashboard();
  void updateConfigsFromDashboard();
  void updateTelemetry();

  ctre::phoenix6::hardware::TalonFX* mFLDrive;
  ctre::phoenix6::hardware::TalonFX* mFLSteer;
  ctre::phoenix6::hardware::CANcoder* mFLEncoder;

  ctre::phoenix6::hardware::TalonFX* mFRDrive;
  ctre::phoenix6::hardware::TalonFX* mFRSteer;
  ctre::phoenix6::hardware::CANcoder* mFREncoder;

  ctre::phoenix6::controls::DutyCycleOut mDriveDutyCycleControl{0.0};
  ctre::phoenix6::controls::PositionVoltage mSteerPositionControl{0_tr};
  ctre::phoenix6::controls::NeutralOut mSteerNeutralControl{};

  bool mFLAtTarget{false};
  bool mFRAtTarget{false};

  double mP;
  double mI;
  double mD;
  double mS;
  units::angle::degree_t mSteerTolerance;

  units::angle::degree_t mTargetSteerAngle{0.0_deg};

  double mFLMagnetOffset;
  double mFRMagnetOffset;

  bool mFLDriveInverted;
  bool mFRDriveInverted;
  bool mFLSteerInverted;
  bool mFRSteerInverted;

  double mSteerGearRatio;
  units::angle::degree_t mMaxSteerAngle;
  double mSpeedScale;
};
