// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include "subsystems/SubDrivetrain.h"
#include "Constants.h"
#include "subsystems/IMU.h"
#include "subsystems/Limelight.h"
#include <frc2/command/button/CommandXboxController.h>

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc2::CommandXboxController m_driverController;
  void ConfigureBindings();
  SubDrivetrain* mSubDriveTrain = nullptr;
  IMU* mSubIMU = nullptr;
};
