// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>

#include "commands/Drive.h"

RobotContainer::RobotContainer() {
  m_Drivetrain = new SubDrivetrain;

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

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
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return frc2::cmd::Print("There is no autonomous command configured");
}
