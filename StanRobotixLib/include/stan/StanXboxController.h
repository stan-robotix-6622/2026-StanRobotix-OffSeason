#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <frc/MathUtil.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Trigger.h>

namespace stan {

class StanXboxController : public frc2::CommandXboxController {
 public:
  explicit StanXboxController(int iPort, double iDeadband = 0.05)
      : frc2::CommandXboxController{iPort}, mDeadband{iDeadband} {}

  double getLeftXWithDeadband(bool iSquared = false) const {
    return applyDeadband(GetLeftX(), mDeadband, iSquared);
  }

  double getLeftYWithDeadband(bool iSquared = false) const {
    return applyDeadband(GetLeftY(), mDeadband, iSquared);
  }

  double getRightXWithDeadband(bool iSquared = false) const {
    return applyDeadband(GetRightX(), mDeadband, iSquared);
  }

  double getRightYWithDeadband(bool iSquared = false) const {
    return applyDeadband(GetRightY(), mDeadband, iSquared);
  }

  double getLeftTriggerWithDeadband(bool iSquared = false) const {
    return applyDeadband(GetLeftTriggerAxis(), mDeadband, iSquared);
  }

  double getRightTriggerWithDeadband(bool iSquared = false) const {
    return applyDeadband(GetRightTriggerAxis(), mDeadband, iSquared);
  }

  void setDeadband(double iDeadband) {
    mDeadband = iDeadband;
  }

  double getDeadband() const {
    return mDeadband;
  }

  void bindHold(frc2::Trigger iTrigger, frc2::CommandPtr iCommand) {
    iTrigger.WhileTrue(std::move(iCommand));
  }

  void bindToggle(frc2::Trigger iTrigger, frc2::CommandPtr iCommand) {
    iTrigger.ToggleOnTrue(std::move(iCommand));
  }

  void bindPress(frc2::Trigger iTrigger, frc2::CommandPtr iCommand) {
    iTrigger.OnTrue(std::move(iCommand));
  }

  static double applyDeadband(double iInput, double iThreshold, bool iSquared = false) {
    double deadbanded = frc::ApplyDeadband(iInput, iThreshold, 1.0);
    if (iSquared) {
      return std::copysign(deadbanded * deadbanded, deadbanded);
    }
    return deadbanded;
  }

 private:
  double mDeadband{0.05};
};

using StanXbox = StanXboxController;

} // namespace stan
