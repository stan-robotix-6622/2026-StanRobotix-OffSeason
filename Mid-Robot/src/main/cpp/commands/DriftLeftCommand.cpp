#include "commands/DriftLeftCommand.h"

DriftLeftCommand::DriftLeftCommand(DriveTrain* driveTrain, std::function<double()> throttleSupplier)
    : mDriveTrain(driveTrain), mThrottleSupplier(std::move(throttleSupplier)) {
  AddRequirements(driveTrain);
}

void DriftLeftCommand::Initialize() {
  mDriveTrain->DriftLeft(mThrottleSupplier());
}

void DriftLeftCommand::Execute() {
  mDriveTrain->DriftLeft(mThrottleSupplier());
}

void DriftLeftCommand::End(bool interrupted) {
  mDriveTrain->Stop();
}

bool DriftLeftCommand::IsFinished() {
  return false;
}
