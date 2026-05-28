// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/sim/SparkMaxSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>

#include "Constants.h"

class SubDrivetrain : public frc2::SubsystemBase {
 public:
  SubDrivetrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  // void setSpeed();
  // units::meters_per_second_t getSpeed();
  void DriveRobot(double iLSpeed, double iRotation);

  double GetEncoderPosition();
  
  void ResetEncoders();

  void Stop();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::spark::SparkMax* mLeftMotorController;
  rev::spark::SparkMax* mRightMotorController;
  frc::DifferentialDrive* mDifferentialDrive;
  frc::DCMotor* mGearBoxL;
  frc::DCMotor* mGearBoxR;
  rev::spark::SparkMaxSim* mLeftMotorControllerSim;
  rev::spark::SparkMaxSim* mRightMotorControllerSim;
  frc::sim::DifferentialDrivetrainSim* mDrivetrainSim;

  //rev::spark::SparkBaseConfig* mSparkBaseConfig;

};