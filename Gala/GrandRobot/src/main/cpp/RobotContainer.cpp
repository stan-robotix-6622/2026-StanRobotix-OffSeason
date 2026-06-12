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
	mSubLED = new SubLED{};

  ConfigureBindings();

  mSubLED->SetDefaultCommand(mSubLED->Run([this] {
	if (abs(mDriverXboxController->GetLeftX()) > 0.2 || abs(mDriverXboxController->GetLeftY()) > 0.2 || abs(mDriverXboxController->GetRightX()) > 0.2) {
	// if (abs(mCommandXboxController->GetLeftX()) > 0.2 || abs(mCommandXboxController->GetLeftY()) > 0.2 || abs(mCommandXboxController->GetRightX()) > 0.2) {
		if (!(mSubLED->mMode == SubLED::Mode::moving)) {
			std::cout << "moving\n";
		}
		mSubLED->setMode(SubLED::Mode::moving);
	}

	else if (mDriverXboxController->GetHID().GetRawButton(OperatorConstants::Button::A)) {
	// else if (mCommandXboxController->Button(4).Get()) {
		if (!(mSubLED->mMode == SubLED::Mode::waving)) {
			std::cout << "waving\n";
		}
		mSubLED->setMode(SubLED::Mode::waving);
	}

	else if (mDriverXboxController->GetHID().GetRawButton(OperatorConstants::Button::B)) {
	// else if (mCommandXboxController->Button(6).Get()) {
		if (!(mSubLED->mMode == SubLED::Mode::test)) {
			std::cout << "test\n";
		}
		mSubLED->setMode(SubLED::Mode::test);
	}
	else {
		mSubLED->setMode(SubLED::Mode::immobile);
	}}));
}

void RobotContainer::ConfigureBindings() 
{
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  mDrivetrain->SetDefaultCommand(
      // Drivetrain will execute this command periodically
    
      mDrivetrain->ApplyRequest([this]() -> auto&& {
          return drive.WithVelocityX(-mDriverXboxController->GetLeftY() * DrivetrainConstants::kMaxDesiredSpeed) // Drive forward with negative Y (forward)
              .WithVelocityY(-mDriverXboxController->GetLeftX() * DrivetrainConstants::kMaxDesiredSpeed) // Drive left with negative X (left)
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
