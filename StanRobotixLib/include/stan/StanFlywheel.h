#pragma once

#include <algorithm>
#include <cmath>
#include <string_view>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include "StanMotor.h"

namespace stan {

struct FlywheelConfig {
  units::angular_velocity::turns_per_second_t kMaxVelocity{100_tps};
  units::angular_velocity::turns_per_second_t kTolerance{2_tps};
  double kP{0.1};
  double kI{0.0};
  double kD{0.0};
  double kS{0.05};
  double kV{0.12};
};

class StanFlywheel : public frc2::SubsystemBase {
 public:
  StanFlywheel(StanMotor* iMotor, const FlywheelConfig& iConfig)
      : mMotor{iMotor}, mOwnsMotor{false}, mConfig{iConfig} {
    configureMotor();
  }

  StanFlywheel(int iCanId, MotorType iType, const FlywheelConfig& iConfig, std::string_view iCanBus = "rio")
      : mMotor{new StanMotor{iCanId, iType, iCanBus}}, mOwnsMotor{true}, mConfig{iConfig} {
    configureMotor();
  }

  ~StanFlywheel() override {
    if (mOwnsMotor) {
      delete mMotor;
    }
  }

  void setVelocity(units::angular_velocity::turns_per_second_t iVelocity) {
    mTargetVelocity = std::clamp(iVelocity, -mConfig.kMaxVelocity, mConfig.kMaxVelocity);
    mMotor->setVelocity(mTargetVelocity);
  }

  void stop() {
    mTargetVelocity = 0_tps;
    mMotor->stopMotor();
  }

  bool atDesiredVelocity() const {
    return units::math::abs(getVelocity() - mTargetVelocity) <= mConfig.kTolerance;
  }

  units::angular_velocity::turns_per_second_t getVelocity() const {
    return mMotor->getVelocity();
  }

  units::angular_velocity::turns_per_second_t getTargetVelocity() const {
    return mTargetVelocity;
  }

  void setTolerance(units::angular_velocity::turns_per_second_t iTolerance) {
    mConfig.kTolerance = iTolerance;
  }

  StanMotor* getMotor() {
    return mMotor;
  }

  frc2::CommandPtr spinVelocity(units::angular_velocity::turns_per_second_t iVelocity) {
    return Run([this, iVelocity] { setVelocity(iVelocity); });
  }

  frc2::CommandPtr spinUpCommand() {
    return Run([this] { setVelocity(mConfig.kMaxVelocity); });
  }

  frc2::CommandPtr stopCommand() {
    return RunOnce([this] { stop(); });
  }

  frc2::CommandPtr waitUntilReady() {
    return frc2::cmd::WaitUntil([this] { return atDesiredVelocity(); });
  }

  void Periodic() override {
    updateConfigsFromDashboard();
    updateTelemetry();
  }

 private:
  void updateConfigsFromDashboard() {}
  void updateTelemetry() {}

  void configureMotor() {
    if (auto* talon = mMotor->getTalonFX()) {
      ctre::phoenix6::configs::Slot0Configs slot0{};
      slot0.kP = mConfig.kP;
      slot0.kI = mConfig.kI;
      slot0.kD = mConfig.kD;
      slot0.kS = mConfig.kS;
      slot0.kV = mConfig.kV;
      talon->GetConfigurator().Apply(slot0);
    }
#if STAN_HAS_REV
    else if (auto* spark = mMotor->getSparkMax()) {
      rev::spark::SparkMaxConfig config{};
      config.closedLoop.Pid(mConfig.kP, mConfig.kI, mConfig.kD);
      config.closedLoop.feedForward.kS(mConfig.kS).kV(mConfig.kV);
      spark->Configure(config,
                       rev::ResetMode::kNoResetSafeParameters,
                       rev::PersistMode::kPersistParameters);
    } else if (auto* sparkFlex = mMotor->getSparkFlex()) {
      rev::spark::SparkFlexConfig config{};
      config.closedLoop.Pid(mConfig.kP, mConfig.kI, mConfig.kD);
      config.closedLoop.feedForward.kS(mConfig.kS).kV(mConfig.kV);
      sparkFlex->Configure(config,
                           rev::ResetMode::kNoResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
    }
#endif
  }

  StanMotor* mMotor{nullptr};
  bool mOwnsMotor{false};
  FlywheelConfig mConfig;
  units::angular_velocity::turns_per_second_t mTargetVelocity{0_tps};
};

} // namespace stan
