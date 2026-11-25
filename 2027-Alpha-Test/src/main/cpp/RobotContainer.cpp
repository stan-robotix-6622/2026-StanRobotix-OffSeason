// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.hpp"

#include <wpi/commands2/Commands.hpp>
// #include <wpi/smartdashboard/SmartDashboard.hpp>

#include "Constants.hpp"

RobotContainer::RobotContainer()
{
  mDriverController = new wpi::cmd::CommandGamepad{OperatorConstants::kDriverControllerPort};

  mDrivetrain = new SubDrivetrain{};

  // wpi::SmartDashboard::PutData(mDrivetrain);

  mDrivetrain->SetDefaultCommand(mDrivetrain->Run([this] {mDrivetrain->driveFieldRelative(
    mDriverController->GetLeftX(),
    mDriverController->GetLeftY(),
    mDriverController->GetRightX(),
    1 - mDriverController->GetRightTrigger());}));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

wpi::cmd::CommandPtr RobotContainer::GetAutonomousCommand() {
  return wpi::cmd::Print("No autonomous command configured");
}
