// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/Commands.h>
#include <functional>

#include "Telemetry.h"
#include "generated/CommandSwerveDrivetrain.h"

class SubDrivetrain : public frc2::SubsystemBase {
 public:
  SubDrivetrain();

  frc2::CommandPtr driveFieldRelativeCommand(std::function<double()> iX, std::function<double()> iY, std::function<double()> i0, double iSpeedModulation);

  void SeedFieldCentric();

  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction iDirection);
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction iDirection);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  units::meters_per_second_t MaxSpeed = 0.3 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
  units::radians_per_second_t MaxAngularRate = 0.5_tps; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
      .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
  swerve::requests::SwerveDriveBrake brake{};
  swerve::requests::PointWheelsAt point{};

  /* Note: This must be constructed before the drivetrain, otherwise we need to
    *       define a destructor to un-register the telemetry from the drivetrain */
  Telemetry logger{MaxSpeed};

  frc2::CommandPtr mRequestedCommand = frc2::cmd::None();

  subsystems::CommandSwerveDrivetrain* mCommandSwerveDrivetrain;
};
