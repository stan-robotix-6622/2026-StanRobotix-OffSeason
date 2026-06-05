// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <iostream>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  m_Drivetrain = new SubDrivetrain;
  // Configure the button bindings
  ConfigureBindings();
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
				m_Drivetrain->DriveRobot(
          -(m_XboxController.GetLeftY() * DriveConstants::kSpeed * (1.2 -m_XboxController.GetRightTriggerAxis())),
          -(m_XboxController.GetRightX() * DriveConstants::kRotationRate * (1.2 - m_XboxController.GetRightTriggerAxis()))
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
