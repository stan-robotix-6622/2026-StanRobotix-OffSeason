#pragma once

#include <algorithm>
#include <string_view>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <units/current.h>
#include <units/voltage.h>
#include "StanMotor.h"

namespace stan {

class StanRoller : public frc2::SubsystemBase {
 public:
  explicit StanRoller(StanMotor* iMotor)
      : mMotor{iMotor}, mOwnsMotor{false} {}

  StanRoller(int iCanId, MotorType iType, std::string_view iCanBus = "rio")
      : mMotor{new StanMotor{iCanId, iType, iCanBus}}, mOwnsMotor{true} {}

  ~StanRoller() override {
    if (mOwnsMotor) {
      delete mMotor;
    }
  }

  void setSpeed(double iSpeed) {
    mTargetSpeed = std::clamp(iSpeed, -1.0, 1.0);
    mMotor->set(mTargetSpeed);
  }

  void setVoltage(units::voltage::volt_t iVoltage) {
    mMotor->setVoltage(iVoltage);
  }

  void stop() {
    mTargetSpeed = 0.0;
    mMotor->stopMotor();
  }

  double getSpeed() const {
    return mTargetSpeed;
  }

  units::current::ampere_t getCurrent() const {
    return mMotor->getCurrent();
  }

  units::voltage::volt_t getVoltage() const {
    return mMotor->getVoltage();
  }

  StanMotor* getMotor() {
    return mMotor;
  }

  frc2::CommandPtr runRoller(double iSpeed) {
    return Run([this, iSpeed] { setSpeed(iSpeed); });
  }

  frc2::CommandPtr runOnce(double iSpeed) {
    return RunOnce([this, iSpeed] { setSpeed(iSpeed); });
  }

  frc2::CommandPtr runEnd(double iSpeed) {
    return frc2::cmd::RunEnd(
        [this, iSpeed] { setSpeed(iSpeed); },
        [this] { stop(); },
        {this});
  }

  frc2::CommandPtr runVoltage(units::voltage::volt_t iVoltage) {
    return Run([this, iVoltage] { setVoltage(iVoltage); });
  }

  frc2::CommandPtr stopCommand() {
    return RunOnce([this] { stop(); });
  }

  void Periodic() override {
    updateConfigsFromDashboard();
    updateTelemetry();
  }

 private:
  void updateConfigsFromDashboard() {}
  void updateTelemetry() {}

  StanMotor* mMotor{nullptr};
  bool mOwnsMotor{false};
  double mTargetSpeed{0.0};
};

} // namespace stan
