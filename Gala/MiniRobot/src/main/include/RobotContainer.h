// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc/Joystick.h>

#include "Constants.h"
#include "subsystems/SubDrivetrain.h"
<<<<<<< HEAD
#include "subsystems/SubLEDs.h"
#include "commands/Drive.h"
#include "commands/DriftL.h"
#include "commands/DriftR.h"
=======
>>>>>>> fc0c6ed111b17d4e621f6d159f8195a26afc2d1d
/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController mXboxController{OperatorConstants::kDriverControllerPort};

<<<<<<< HEAD
  // The robot's subsystems are defined here...
  ExampleSubsystem m_subsystem;
  SubDrivetrain* m_Drivetrain;
  SubLEDs mLED;
  

  double m_currentSpeed = 0.0;
  double m_currentRotation = 0.0;

  frc::Joystick* mJoystick;
=======
  SubDrivetrain* mDrivetrain;

  double mCurrentSpeed = 0.0;
  double mCurrentRotation = 0.0;
>>>>>>> fc0c6ed111b17d4e621f6d159f8195a26afc2d1d
  
  void ConfigureBindings();
};
