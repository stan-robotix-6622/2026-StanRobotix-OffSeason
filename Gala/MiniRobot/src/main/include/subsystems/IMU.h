// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include "Constants.h"

class IMU : public frc2::SubsystemBase {
 public:
  IMU();

	void reset();
	frc::Rotation2d getRotation2d();
	units::degree_t getAngleYaw();
	units::degrees_per_second_t getYawRate();
	
	void setAngleYaw(units::degree_t iAngle);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:

 ctre::phoenix6::hardware::Pigeon2 *mIMU;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
