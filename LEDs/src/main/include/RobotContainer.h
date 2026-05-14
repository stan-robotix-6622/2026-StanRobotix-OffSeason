// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/CommandPtr.h>

#include "SubLEDs.h"

class RobotContainer {
 public:
	RobotContainer();

	frc2::CommandPtr GetAutonomousCommand();

 private:
	void ConfigureBindings();

	frc2::CommandXboxController m_driverController{
		OperatorConstants::kDriverControllerPort};

	SubLEDs mLED;
	frc2::CommandJoystick* m_commandJoystick;
	frc::Joystick* m_joystick;
};
