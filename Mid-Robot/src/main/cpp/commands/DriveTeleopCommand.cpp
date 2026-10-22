#include "commands/DriveTeleopCommand.h"

DriveTeleopCommand::DriveTeleopCommand(DriveTrain* driveTrain,
                                       std::function<double()> throttleSupplier,
                                       std::function<double()> brakeSupplier,
                                       std::function<double()> steerSupplier,
                                       std::function<bool()> reverseSupplier,
                                       std::function<bool()> driftSupplier)
    : mDriveTrain(driveTrain),
      mThrottleSupplier(std::move(throttleSupplier)),
      mBrakeSupplier(std::move(brakeSupplier)),
      mSteerSupplier(std::move(steerSupplier)),
      mReverseSupplier(std::move(reverseSupplier)),
      mDriftSupplier(std::move(driftSupplier)) {
  AddRequirements(driveTrain);
}

void DriveTeleopCommand::Execute() {
  mDriveTrain->Drive(mThrottleSupplier(),
                     mBrakeSupplier(),
                     mSteerSupplier(),
                     mReverseSupplier(),
                     mDriftSupplier());
}

void DriveTeleopCommand::End(bool interrupted) {
  mDriveTrain->Stop();
}
