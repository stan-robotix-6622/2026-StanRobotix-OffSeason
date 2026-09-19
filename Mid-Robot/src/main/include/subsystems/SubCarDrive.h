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
  ctre::phoenix6::hardware::TalonFX* mFrontLeftDrive;
  ctre::phoenix6::hardware::TalonFX* mFrontLeftSteer;
  ctre::phoenix6::hardware::CANcoder* mFrontLeftCANcoder;

  ctre::phoenix6::hardware::TalonFX* mFrontRightDrive;
  ctre::phoenix6::hardware::TalonFX* mFrontRightSteer;
  ctre::phoenix6::hardware::CANcoder* mFrontRightCANcoder;

  ctre::phoenix6::controls::DutyCycleOut mDriveDutyCycleControl{0.0};
  ctre::phoenix6::controls::PositionVoltage mSteerPositionControl{0_tr};
};
