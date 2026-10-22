#pragma once

#include <functional>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveTrain.h"

class DriveTeleopCommand : public frc2::CommandHelper<frc2::Command, DriveTeleopCommand> {
 public:
  DriveTeleopCommand(DriveTrain* driveTrain,
                     std::function<double()> throttleSupplier,
                     std::function<double()> brakeSupplier,
                     std::function<double()> steerSupplier,
                     std::function<bool()> reverseSupplier,
                     std::function<bool()> driftSupplier = [] { return false; });

  void Execute() override;
  void End(bool interrupted) override;

 private:
  DriveTrain* mDriveTrain;
  std::function<double()> mThrottleSupplier;
  std::function<double()> mBrakeSupplier;
  std::function<double()> mSteerSupplier;
  std::function<bool()> mReverseSupplier;
  std::function<bool()> mDriftSupplier;
};
