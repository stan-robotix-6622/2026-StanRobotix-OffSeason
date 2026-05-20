// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

#include <iostream>

RobotContainer::RobotContainer()
{
	// mCommandXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
	mJoystick = new frc::Joystick{OperatorConstants::kDriverControllerPort};

	ConfigureBindings();

	mLED.SetDefaultCommand(mLED.Run([this] {
	if (abs(mJoystick->GetX()) > 0.2 || abs(mJoystick->GetY()) > 0.2 || abs(mJoystick->GetZ()) > 0.2) {
	// if (abs(mCommandXboxController->GetLeftX()) > 0.2 || abs(mCommandXboxController->GetLeftY()) > 0.2 || abs(mCommandXboxController->GetRightX()) > 0.2) {
		if (!(mLED.mMode == SubLEDs::Mode::moving)) {
			std::cout << "moving\n";
		}
		mLED.setMode(SubLEDs::Mode::moving);
	}

	else if (mJoystick->GetRawButton(4)) {
	// else if (mCommandXboxController->Button(4).Get()) {
		if (!(mLED.mMode == SubLEDs::Mode::waving)) {
			std::cout << "waving\n";
		}
		mLED.setMode(SubLEDs::Mode::waving);
	}

	else if (mJoystick->GetRawButton(6)) {
	// else if (mCommandXboxController->Button(6).Get()) {
		if (!(mLED.mMode == SubLEDs::Mode::test)) {
			std::cout << "test\n";
		}
		mLED.setMode(SubLEDs::Mode::test);
	}
	else {
		mLED.setMode(SubLEDs::Mode::immobile);
	}}));
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
	return frc2::cmd::Print("No autonomous command configured");
}