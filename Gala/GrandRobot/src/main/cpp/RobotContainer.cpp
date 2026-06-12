// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <iostream>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() 
{
  mDriverXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  mDrivetrain = new SubDrivetrain{};
  mSubLED = new SubLED{};

  ConfigureBindings();

  mDrivetrain->SetDefaultCommand(mDrivetrain->driveFieldRelativeCommand(-mDriverXboxController->GetLeftY(), -mDriverXboxController->GetLeftX(), -mDriverXboxController->GetRightX(), 1));

  mSubLED->SetDefaultCommand(mSubLED->Run([this] {
			if (abs(mDriverXboxController->GetLeftX()) > 0.2 || abs(mDriverXboxController->GetLeftY()) > 0.2 || abs(mDriverXboxController->GetRightX()) > 0.2) {
				if (mSubLED->mMode != SubLED::moving) {
					std::cout << "moving\n";
				}
				mSubLED->setMode(SubLED::moving);
			}
			else if (mDriverXboxController->Button(OperatorConstants::Button::A).Get()) {
				if (mSubLED->mMode != SubLED::waving) {
					std::cout << "waving\n";
				}
				mSubLED->setMode(SubLED::waving);
			}
			else if (mDriverXboxController->Button(OperatorConstants::Button::B).Get()) {
				if (mSubLED->mMode != SubLED::test) {
					std::cout << "test\n";
				}
				mSubLED->setMode(SubLED::test);
			}
			else {
				mSubLED->setMode(SubLED::immobile);
			}
	}));
}

void RobotContainer::ConfigureBindings() 
{
  // Run SysId routines when holding back/start and X/Y.
  // Note that each routine should be run exactly once in a single log.
  (mDriverXboxController->Back() && mDriverXboxController->Y()).WhileTrue(mDrivetrain->SysIdDynamic(frc2::sysid::Direction::kForward));
  (mDriverXboxController->Back() && mDriverXboxController->X()).WhileTrue(mDrivetrain->SysIdDynamic(frc2::sysid::Direction::kReverse));
  (mDriverXboxController->Start() && mDriverXboxController->Y()).WhileTrue(mDrivetrain->SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (mDriverXboxController->Start() && mDriverXboxController->X()).WhileTrue(mDrivetrain->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // reset the field-centric heading on left bumper press
  mDriverXboxController->LeftBumper().OnTrue(mDrivetrain->RunOnce([this] { mDrivetrain->SeedFieldCentric(); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  return frc2::cmd::Print("No autonomous command configured");
}
