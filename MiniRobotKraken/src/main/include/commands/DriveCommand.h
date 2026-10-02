// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/controller/PIDController.h>
#include "subsystems/DriveTrain.h"
#include "DriveCommand.h"
#include "Constants.h"
/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveCommand
    : public frc2::CommandHelper<frc2::Command, DriveCommand> {
 public:
  /* You should consider using the more terse Command factories API instead
   * https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands
   */
	explicit DriveCommand(DriveTrain* iDriveCommand, frc2::CommandXboxController* iXboxController);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;



	private:
	frc::PIDController* mPIDController;
  frc2::CommandXboxController* mXboxController;
	DriveTrain* mDriveCommand;
};
