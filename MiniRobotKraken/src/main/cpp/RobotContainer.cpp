// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/RunCommand.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  mXboxController = new frc::XboxController{0};
//	mDriveTrainAngle = new DriveTrainAngle;
  
  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());

 mDriveTrain->SetDefaultCommand(frc2::RunCommand(
    [this] {
<<<<<<< HEAD
     if (abs(mXboxController->GetLeftX()) > 0.1 || abs(mXboxController->GetRightY()) > 0.1) {
       mDriveTrain->Drive(mXboxController->GetRightY(), mXboxController->GetLeftX());
     }
      else {
       mDriveTrain->Drive(0, 0);
     }
   }, {mDriveTrain}
 ));


=======
      if (abs(mXboxController->GetLeftX()) > 0.1 || abs(mXboxController->GetRightY()) > 0.1) {
        mDriveTrain->Drive(mXboxController->GetRightY(), mXboxController->GetLeftX());
      }

      else {
        mDriveTrain->Drive(0, 0);
      }
    }, {mDriveTrain}
  ));
>>>>>>> 1981151652aa4bb23806005ff76cc07305ddd370
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
