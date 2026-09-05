// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/JoystickButton.h>
#include <iostream>

#include "commands/Drive.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  mDrivetrain = new SubDrivetrain;

  // mJoystick = new frc::Joystick{OperatorConstants::kDriverJoystickPort};
  // Configure the button bindings
  ConfigureBindings();

  	mLED.SetDefaultCommand(mLED.Run([this] {
	// if (abs(mJoystick->GetX()) > 0.2 || abs(mJoystick->GetY()) > 0.2 || abs(mJoystick->GetZ()) > 0.2) {
	if (abs(mXboxController.GetLeftX()) > 0.2 || abs(mXboxController.GetLeftY()) > 0.2 || abs(mXboxController.GetRightX()) > 0.2) {
		if (!(mLED.mMode == SubLEDs::Mode::moving)) {
			std::cout << "moving\n";
		}
		mLED.setMode(SubLEDs::Mode::moving);
	}

	//else if (mJoystick->GetRawButton(4)) { // Bouton Y
	else if (mXboxController.Button(4).Get()) { // Bouton Y
		if (!(mLED.mMode == SubLEDs::Mode::waving)) {
			std::cout << "waving\n";
		}
		mLED.setMode(SubLEDs::Mode::waving);
	}

	// else if (mJoystick->GetRawButton(6)) { // Bouton RB
	else if (mXboxController.Button(6).Get()) { // Bouton RB
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
  std::cout << "Test\n";

  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
			[this] {
        double targetSpeed = mXboxController.GetLeftY();
        double targetRotation = mXboxController.GetRightX();

        std::cout << "Left Y : " << targetSpeed << std::endl;
        std::cout << "Right X : " << targetRotation << std::endl;
                

        mCurrentSpeed += (targetSpeed - mCurrentSpeed) * DriveConstants::kSmooth;
        mCurrentRotation += (targetRotation - mCurrentRotation) * DriveConstants::kSmooth;

				mDrivetrain->DriveRobot(
          -(mCurrentSpeed * DriveConstants::kSpeed * (1.2 -mXboxController.GetRightTriggerAxis())),
          -(mCurrentRotation * DriveConstants::kRotationRate * (1.2 - mXboxController.GetRightTriggerAxis()))
        );
			},
			{mDrivetrain}));

  mXboxController.A().WhileTrue(frc2::cmd::Run(
    [this] {
      mDrivetrain->Stop();
    },
    {mDrivetrain}));

  mXboxController.LeftBumper().WhileTrue(DriftL(mDrivetrain).ToPtr());
  mXboxController.RightBumper().WhileTrue(DriftR(mDrivetrain).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no autonomous command configured");
}
