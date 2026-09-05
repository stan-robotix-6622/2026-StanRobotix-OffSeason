// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
<<<<<<< HEAD
#include <frc2/command/button/JoystickButton.h>
#include <iostream>
=======
>>>>>>> fc0c6ed111b17d4e621f6d159f8195a26afc2d1d

#include "commands/Drive.h"

RobotContainer::RobotContainer() {
<<<<<<< HEAD
  // Initialize all of your commands and subsystems here
  m_Drivetrain = new SubDrivetrain;

  // mJoystick = new frc::Joystick{OperatorConstants::kDriverJoystickPort};
=======
  mDrivetrain = new SubDrivetrain;

>>>>>>> fc0c6ed111b17d4e621f6d159f8195a26afc2d1d
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

<<<<<<< HEAD
  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  std::cout << "Test\n";

  m_Drivetrain->SetDefaultCommand(frc2::cmd::Run(
			[this] {
        double targetSpeed = m_XboxController.GetLeftY();
        double targetRotation = m_XboxController.GetRightX();

        std::cout << "Left Y : " << targetSpeed << std::endl;
        std::cout << "Right X : " << targetRotation << std::endl;
                

        m_currentSpeed += (targetSpeed - m_currentSpeed) * DriveConstants::kSmooth;
        m_currentRotation += (targetRotation - m_currentRotation) * DriveConstants::kSmooth;

				m_Drivetrain->DriveRobot(
          -(m_currentSpeed * DriveConstants::kSpeed * (1.2 -m_XboxController.GetRightTriggerAxis())),
          -(m_currentRotation * DriveConstants::kRotationRate * (1.2 - m_XboxController.GetRightTriggerAxis()))
=======
  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
			[this] {
        double targetSpeed = mXboxController.GetLeftY();
        double targetRotation = mXboxController.GetRightX();

        mCurrentSpeed += (targetSpeed - mCurrentSpeed) * DriveConstants::kSmooth;
        mCurrentRotation += (targetRotation - mCurrentRotation) * DriveConstants::kSmooth;

				mDrivetrain->DriveRobot(
          -(mCurrentSpeed * DriveConstants::kSpeed * (1.2 -mXboxController.GetRightTriggerAxis())),
          -(mCurrentRotation * DriveConstants::kRotationRate * (1.2 - mXboxController.GetRightTriggerAxis()))
>>>>>>> fc0c6ed111b17d4e621f6d159f8195a26afc2d1d
        );
			},
			{mDrivetrain}));

  mXboxController.A().WhileTrue(frc2::cmd::Run(
    [this] {
      mDrivetrain->Stop();
    },
<<<<<<< HEAD
    {m_Drivetrain}));

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_XboxController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

  m_XboxController.LeftBumper().WhileTrue(DriftL(m_Drivetrain).ToPtr());
  m_XboxController.RightBumper().WhileTrue(DriftR(m_Drivetrain).ToPtr());
=======
    {mDrivetrain}));
>>>>>>> fc0c6ed111b17d4e621f6d159f8195a26afc2d1d
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no autonomous command configured");
}
