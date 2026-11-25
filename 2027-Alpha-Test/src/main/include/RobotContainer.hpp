// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/button/CommandGamepad.hpp>

#include "subsystems/SubDrivetrain.hpp"

class RobotContainer {
 public:
  RobotContainer();

  wpi::cmd::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();

  wpi::cmd::CommandGamepad* mDriverController;

  SubDrivetrain* mDrivetrain;
};
