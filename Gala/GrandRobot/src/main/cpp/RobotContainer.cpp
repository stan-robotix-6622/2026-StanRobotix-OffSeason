// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <iostream>

#include <frc2/command/button/Trigger.h>

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include "RobotixLib.hpp"

RobotContainer::RobotContainer()
{
	mDriverXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};

	mDrivetrain = new SubDrivetrain{};
	mLEDs = new SubLEDs{};

	ConfigureBindings();

  mDrivetrain->SetDefaultCommand(mDrivetrain->driveFieldRelativeCommand(
      [this] { return robotixLib::deadband(-mDriverXboxController->GetLeftY(), 0.05); },
      [this] { return robotixLib::deadband(-mDriverXboxController->GetLeftX(), 0.05); },
      [this] { return robotixLib::deadband(-mDriverXboxController->GetRightX(), 0.05); },
      [this] {
					if (mDriverXboxController->GetHID().GetRawButtonPressed(robotixLib::Xbox::Button::LeftBumper)) {
						mToggleFastDrivetrain = !mToggleFastDrivetrain;
					} 
					return mToggleFastDrivetrain ? 1.0 : 0.2;}
  ));

	mLEDs->SetDefaultCommand(mLEDs->Run([this] {
	if (abs(mDriverXboxController->GetLeftX()) > 0.2 || abs(mDriverXboxController->GetLeftY()) > 0.2 || abs(mDriverXboxController->GetRightX()) > 0.2) {
		if (mLEDs->getMode() != SubLEDs::Mode::moving) {
			std::cout << "moving\n";
			mLEDs->setMode(SubLEDs::Mode::moving);
		}
	}

	else if (mDriverXboxController->Button(robotixLib::Xbox::Button::Y).Get()) {
		if (mLEDs->getMode() != SubLEDs::Mode::deploying) {
			std::cout << "deploying\n";
			mLEDs->setMode(SubLEDs::Mode::deploying);
		}
	}

	else if (mDriverXboxController->Button(robotixLib::Xbox::Button::RightBumper).Get()) {
		if (mLEDs->getMode() != SubLEDs::Mode::test) {
			std::cout << "test\n";
			mLEDs->setMode(SubLEDs::Mode::test);
		}
	}
	else {
		if (mLEDs->getMode() != SubLEDs::Mode::immobile) {
			std::cout << "immobile\n";
			mLEDs->setMode(SubLEDs::Mode::immobile);
		}
	}}));
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
    // Simple drive forward auton
    return frc2::cmd::Print("There is no configured autonomous command");
}
