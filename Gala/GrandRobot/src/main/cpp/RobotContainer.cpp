// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include "RobotixLib.hpp"

RobotContainer::RobotContainer()
{
	mDriverXboxController = new frc2::CommandXboxController{OperatorConstants::kDriverControllerPort};
	ConfigureBindings();

	// mLED.SetDefaultCommand(mLED.Run([this] {
	// if (abs(mDriverXboxController->GetLeftX()) > 0.2 || abs(mDriverXboxController->GetLeftY()) > 0.2 || abs(mDriverXboxController->GetRightX()) > 0.2) {
	// 	if (mLED.mMode != SubLEDs::Mode::moving) {
	// 	//	std::cout << "moving\n";
	// 	}
	// 	mLED.setMode(SubLEDs::Mode::moving);
	// }

	// else if (mDriverXboxController->Button(robotixLib::Xbox::Button::Y).Get()) {
	// 	if (mLED.mMode != SubLEDs::Mode::waving) {
	// 	//	std::cout << "waving\n";
	// 	}
	// 	mLED.setMode(SubLEDs::Mode::waving);
	// }

	// else if (mDriverXboxController->Button(robotixLib::Xbox::Button::RightBumper).Get()) {
	// 	if (mLED.mMode != SubLEDs::Mode::test) {
	// 		//std::cout << "test\n";
	// 	}
	// 	mLED.setMode(SubLEDs::Mode::test);
	// }
	// else {
	// 	mLED.setMode(SubLEDs::Mode::immobile);
	// }}));
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(robotixLib::deadband(-mDriverXboxController->GetLeftY(), 0.05) * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(robotixLib::deadband(-mDriverXboxController->GetLeftX(), 0.05) * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(robotixLib::deadband(-mDriverXboxController->GetRightX(), 0.05) * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );

    mDriverXboxController->A().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    mDriverXboxController->B().WhileTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-mDriverXboxController->GetLeftY(), -mDriverXboxController->GetLeftX()});
    }));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (mDriverXboxController->Back() && mDriverXboxController->Y()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kForward));
    (mDriverXboxController->Back() && mDriverXboxController->X()).WhileTrue(drivetrain.SysIdDynamic(frc2::sysid::Direction::kReverse));
    (mDriverXboxController->Start() && mDriverXboxController->Y()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (mDriverXboxController->Start() && mDriverXboxController->X()).WhileTrue(drivetrain.SysIdQuasistatic(frc2::sysid::Direction::kReverse));

    // reset the field-centric heading on left bumper press
    mDriverXboxController->LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    // Simple drive forward auton
    return frc2::cmd::Sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(0.5_mps)
                .WithVelocityY(0_mps)
                .WithRotationalRate(0_tps);
        })
        .WithTimeout(5_s),
        // Finally idle for the rest of auton
        drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
    );
}
