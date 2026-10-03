// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <ctre/phoenix6/HootReplay.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <iostream>

class DriveTrain : public frc2::SubsystemBase {
 public:
  DriveTrain();
  void Drive(double Turn, double Drive);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
    
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX* mRightMotorDrive;
	ctre::phoenix6::hardware::TalonFX* mRightMotorTurn;
  ctre::phoenix6::hardware::TalonFX* mLeftMotorDrive;
	ctre::phoenix6::hardware::TalonFX* mLeftMotorTurn;
	frc::PIDController* mPIDController;
	ctre::phoenix6::hardware::CANcoder* mLeftEncoder;	
	ctre::phoenix6::hardware::CANcoder* mRightEncoder;	
};



