// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.h"

#include <frc2/command/button/RobotModeTriggers.h>

#include "Constants.h"

SubDrivetrain::SubDrivetrain()
{
  // Note that X is defined as forward according to WPILib convention,
  // and Y is defined as to the left according to WPILib convention.
  mCommandSwerveDrivetrain = new subsystems::CommandSwerveDrivetrain{TunerConstants::CreateDrivetrain()};

  mCommandSwerveDrivetrain->RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
  
  // Idle while the robot is disabled. This ensures the configured
  // neutral mode is applied to the drive motors while disabled.
  frc2::RobotModeTriggers::Disabled().WhileTrue(
      mCommandSwerveDrivetrain->ApplyRequest([] {
          return swerve::requests::Idle{};
      }).IgnoringDisable(true)
  );
}

// This method will be called once per scheduler run
void SubDrivetrain::Periodic() {}

frc2::CommandPtr SubDrivetrain::driveFieldRelativeCommand(std::function<double()> iX, std::function<double()> iY, std::function<double()> i0, std::function<double()> iSpeedModulation)
{
  frc2::CommandPtr mRequestedCommand = mCommandSwerveDrivetrain->ApplyRequest([this, iX, iY, i0, iSpeedModulation]() -> auto&& {
      return drive.WithVelocityX(iX() * MaxSpeed * iSpeedModulation())
          .WithVelocityY(iY() * MaxSpeed * iSpeedModulation())
          .WithRotationalRate(i0() * MaxAngularRate * iSpeedModulation());
  });
  mRequestedCommand.get()->AddRequirements(this);
  return mRequestedCommand;
}

void SubDrivetrain::SeedFieldCentric()
{
  mCommandSwerveDrivetrain->SeedFieldCentric();
}

frc2::CommandPtr SubDrivetrain::SysIdDynamic(frc2::sysid::Direction iDirection)
{
  return mCommandSwerveDrivetrain->SysIdDynamic(iDirection);
}

frc2::CommandPtr SubDrivetrain::SysIdQuasistatic(frc2::sysid::Direction iDirection)
{
  return mCommandSwerveDrivetrain->SysIdQuasistatic(iDirection);
}
