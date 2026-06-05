// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/JoystickButton.h>
#include <iostream>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_Drivetrain = new SubDrivetrain;

  // mJoystick = new frc::Joystick{OperatorConstants::kDriverJoystickPort};
  // Configure the button bindings
  ConfigureBindings();

  	mLED.SetDefaultCommand(mLED.Run([this] {
	// if (abs(mJoystick->GetX()) > 0.2 || abs(mJoystick->GetY()) > 0.2 || abs(mJoystick->GetZ()) > 0.2) {
	if (abs(m_XboxController.GetLeftX()) > 0.2 || abs(m_XboxController.GetLeftY()) > 0.2 || abs(m_XboxController.GetRightX()) > 0.2) {
		if (!(mLED.mMode == SubLEDs::Mode::moving)) {
			std::cout << "moving\n";
		}
		mLED.setMode(SubLEDs::Mode::moving);
	}

	//else if (mJoystick->GetRawButton(4)) { // Bouton Y
	else if (m_XboxController.Button(4).Get()) { // Bouton Y
		if (!(mLED.mMode == SubLEDs::Mode::waving)) {
			std::cout << "waving\n";
		}
		mLED.setMode(SubLEDs::Mode::waving);
	}

	// else if (mJoystick->GetRawButton(6)) { // Bouton RB
	else if (m_XboxController.Button(6).Get()) { // Bouton RB
		if (!(mLED.mMode == SubLEDs::Mode::test)) {
			std::cout << "test\n";
		}
		mLED.setMode(SubLEDs::Mode::test);
	}
	else {
		mLED.setMode(SubLEDs::Mode::immobile);
	}}));
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  std::cout << "Test\n";

  m_Drivetrain->SetDefaultCommand(frc2::cmd::Run(
			[this] {
        double targetSpeed = m_XboxController.GetLeftY();
        double targetRotation = m_XboxController.GetRightX();

        m_currentSpeed += (targetSpeed - m_currentSpeed) * DriveConstants::kSmooth;
        m_currentRotation += (targetRotation - m_currentRotation) * DriveConstants::kSmooth;

				m_Drivetrain->DriveRobot(
          -(m_currentSpeed * DriveConstants::kSpeed * (1.2 -m_XboxController.GetRightTriggerAxis())),
          -(m_currentRotation * DriveConstants::kRotationRate * (1.2 - m_XboxController.GetRightTriggerAxis()))
        );
			},
			{m_Drivetrain}));

  m_XboxController.A().WhileTrue(frc2::cmd::Run(
    [this] {
      m_Drivetrain->Stop();
    },
    {m_Drivetrain}));

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_XboxController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
