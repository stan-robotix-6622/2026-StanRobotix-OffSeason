#include "commands/DriftRightCommand.h"

DriftRightCommand::DriftRightCommand(DriveTrain* driveTrain, std::function<double()> throttleSupplier)
    : mDriveTrain(driveTrain), mThrottleSupplier(std::move(throttleSupplier)) {
  AddRequirements(driveTrain);
}

void DriftRightCommand::Initialize() {
  mDriveTrain->DriftRight(mThrottleSupplier());
}

void DriftRightCommand::Execute() {
  mDriveTrain->DriftRight(mThrottleSupplier());
}

void DriftRightCommand::End(bool interrupted) {
  mDriveTrain->Stop();
}

bool DriftRightCommand::IsFinished() {
  return false;
}
