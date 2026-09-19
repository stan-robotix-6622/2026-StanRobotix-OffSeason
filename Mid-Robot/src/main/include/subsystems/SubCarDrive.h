#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

class SubCarDrive : public frc2::SubsystemBase {
 public:
  SubCarDrive();
  ~SubCarDrive() override;

  void drive(double iThrottle, double iBrake, double iSteer);
  void stop();

  void Periodic() override;

 private:
  ctre::phoenix6::hardware::TalonFX* mFLDrive;
  ctre::phoenix6::hardware::TalonFX* mFLSteer;
  ctre::phoenix6::hardware::CANcoder* mFLEncoder;

  ctre::phoenix6::hardware::TalonFX* mFRDrive;
  ctre::phoenix6::hardware::TalonFX* mFRSteer;
  ctre::phoenix6::hardware::CANcoder* mFREncoder;

  ctre::phoenix6::controls::DutyCycleOut mDriveDutyCycleControl{0.0};
  ctre::phoenix6::controls::PositionVoltage mSteerPositionControl{0_tr};
};
