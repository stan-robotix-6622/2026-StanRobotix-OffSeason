// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

#include "commands/Drive.h"

RobotContainer::RobotContainer() {
  mDrivetrain = new SubDrivetrain;

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  mDrivetrain->SetDefaultCommand(frc2::cmd::Run(
			[this] {
        double targetSpeed = mXboxController.GetLeftY();
        double targetRotation = mXboxController.GetRightX();

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
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no autonomous command configured");
}
