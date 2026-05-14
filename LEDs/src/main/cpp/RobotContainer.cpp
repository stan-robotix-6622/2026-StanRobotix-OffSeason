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
	m_commandJoystick = new frc2::CommandJoystick{OperatorConstants::kDriverControllerPort};
	m_joystick = &m_commandJoystick->GetHID();

	ConfigureBindings();

	mLED.SetDefaultCommand(mLED.Run([this] {
	if (abs(m_joystick->GetX()) > 0.2 || abs(m_joystick->GetY()) > 0.2 || abs(m_joystick->GetZ()) > 0.2) {
		if (!(mLED.mMode == SubLEDs::Mode::moving)) {
			std::cout << "moving\n";
		}
		mLED.setMode(SubLEDs::Mode::moving);
	}

	else if (m_joystick->GetRawButton(4)) {
		if (!(mLED.mMode == SubLEDs::Mode::waving)) {
			std::cout << "waving\n";
		}
		mLED.setMode(SubLEDs::Mode::waving);
	}

	else if (m_joystick->GetRawButton(6)) {
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