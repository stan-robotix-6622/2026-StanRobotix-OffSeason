// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer() 
{
  mDrivetrain = new subsystems::CommandSwerveDrivetrain{TunerConstants::CreateDrivetrain()};
  mDriverXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() 
{
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  mDrivetrain->SetDefaultCommand(
      // Drivetrain will execute this command periodically
      mDrivetrain->ApplyRequest([this]() -> auto&& {
          return drive.WithVelocityX(-mDriverXboxController->GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .WithVelocityY(-mDriverXboxController->GetLeftX() * MaxSpeed) // Drive left with negative X (left)
              .WithRotationalRate(-mDriverXboxController->GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
      })
  );

  // Idle while the robot is disabled. This ensures the configured
  // neutral mode is applied to the drive motors while disabled.
  frc2::RobotModeTriggers::Disabled().WhileTrue(
      mDrivetrain->ApplyRequest([] {
          return swerve::requests::Idle{};
      }).IgnoringDisable(true)
  );

  mDriverXboxController->A().WhileTrue(mDrivetrain->ApplyRequest([this]() -> auto&& { return brake; }));
  mDriverXboxController->B().WhileTrue(mDrivetrain->ApplyRequest([this]() -> auto&& {
      return point.WithModuleDirection(frc::Rotation2d{-mDriverXboxController->GetLeftY(), -mDriverXboxController->GetLeftX()});
  }));

  // Run SysId routines when holding back/start and X/Y.
  // Note that each routine should be run exactly once in a single log.
  (mDriverXboxController->Back() && mDriverXboxController->Y()).WhileTrue(mDrivetrain->SysIdDynamic(frc2::sysid::Direction::kForward));
  (mDriverXboxController->Back() && mDriverXboxController->X()).WhileTrue(mDrivetrain->SysIdDynamic(frc2::sysid::Direction::kReverse));
  (mDriverXboxController->Start() && mDriverXboxController->Y()).WhileTrue(mDrivetrain->SysIdQuasistatic(frc2::sysid::Direction::kForward));
  (mDriverXboxController->Start() && mDriverXboxController->X()).WhileTrue(mDrivetrain->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

  // reset the field-centric heading on left bumper press
  mDriverXboxController->LeftBumper().OnTrue(mDrivetrain->RunOnce([this] { mDrivetrain->SeedFieldCentric(); }));

  mDrivetrain->RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() 
{
  return frc2::cmd::Print("No autonomous command configured");
}
