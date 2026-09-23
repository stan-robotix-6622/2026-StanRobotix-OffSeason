#pragma once

#include <algorithm>
#include <cmath>
#include <string_view>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>
#include <units/math.h>
#include "StanMotor.h"

namespace stan {

struct ElevatorConfig {
  units::length::meter_t kMinHeight{0.0_m};
  units::length::meter_t kMaxHeight{1.5_m};
  units::length::meter_t kTolerance{0.01_m};
  double kMetersPerRotation{0.05};
  double kP{30.0};
  double kI{0.0};
  double kD{0.2};
  double kG{0.4};
  double kS{0.0};
  double kV{0.0};
};

class StanElevator : public frc2::SubsystemBase {
 public:
  StanElevator(StanMotor* iMotor, const ElevatorConfig& iConfig)
      : mMotor{iMotor}, mOwnsMotor{false}, mConfig{iConfig} {
    configureMotor();
  }

  StanElevator(int iCanId, MotorType iType, const ElevatorConfig& iConfig, std::string_view iCanBus = "rio")
      : mMotor{new StanMotor{iCanId, iType, iCanBus}}, mOwnsMotor{true}, mConfig{iConfig} {
    configureMotor();
  }

  ~StanElevator() override {
    if (mOwnsMotor) {
      delete mMotor;
    }
  }

  void setTargetHeight(units::length::meter_t iHeight) {
    units::length::meter_t target = std::clamp(iHeight, mConfig.kMinHeight, mConfig.kMaxHeight);
    mTargetHeight = target;
    units::angle::turn_t turns{target.value() / mConfig.kMetersPerRotation};
    mMotor->setPosition(turns);
  }

  void stop() {
    mMotor->stopMotor();
  }

  bool atTargetHeight() const {
    return units::math::abs(getHeight() - mTargetHeight) <= mConfig.kTolerance;
  }

  units::length::meter_t getHeight() const {
    return units::length::meter_t{mMotor->getPosition().value() * mConfig.kMetersPerRotation};
  }

  units::length::meter_t getTargetHeight() const {
    return mTargetHeight;
  }

  void setTolerance(units::length::meter_t iTolerance) {
    mConfig.kTolerance = iTolerance;
  }

  void zeroHeight() {
    if (auto* talon = mMotor->getTalonFX()) {
      talon->SetPosition(0_tr);
    }
#if STAN_HAS_REV
    else if (auto* encoder = mMotor->getSparkRelativeEncoder()) {
      encoder->SetPosition(0.0);
    }
#endif
    mTargetHeight = 0_m;
  }

  StanMotor* getMotor() {
    return mMotor;
  }

  frc2::CommandPtr goToHeight(units::length::meter_t iHeight) {
    return Run([this, iHeight] { setTargetHeight(iHeight); });
  }

  frc2::CommandPtr holdHeight() {
    return Run([this] { setTargetHeight(mTargetHeight); });
  }

  frc2::CommandPtr zeroCommand() {
    return RunOnce([this] { zeroHeight(); });
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
      ctre::phoenix6::configs::TalonFXConfiguration config{};
      config.Slot0.kP = mConfig.kP;
      config.Slot0.kI = mConfig.kI;
      config.Slot0.kD = mConfig.kD;
      config.Slot0.kS = mConfig.kS;
      config.Slot0.kV = mConfig.kV;
      config.Slot0.kG = mConfig.kG;
      config.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          units::angle::turn_t{mConfig.kMaxHeight.value() / mConfig.kMetersPerRotation};
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
          units::angle::turn_t{mConfig.kMinHeight.value() / mConfig.kMetersPerRotation};
      config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
      talon->GetConfigurator().Apply(config);
    }
#if STAN_HAS_REV
    else if (auto* spark = mMotor->getSparkMax()) {
      rev::spark::SparkMaxConfig config{};
      config.closedLoop.Pid(mConfig.kP, mConfig.kI, mConfig.kD);
      config.closedLoop.feedForward.kS(mConfig.kS).kV(mConfig.kV).kG(mConfig.kG);
      config.softLimit.ForwardSoftLimit(mConfig.kMaxHeight.value() / mConfig.kMetersPerRotation)
          .ForwardSoftLimitEnabled(true);
      config.softLimit.ReverseSoftLimit(mConfig.kMinHeight.value() / mConfig.kMetersPerRotation)
          .ReverseSoftLimitEnabled(true);
      config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
      spark->Configure(config,
                       rev::ResetMode::kNoResetSafeParameters,
                       rev::PersistMode::kPersistParameters);
    } else if (auto* sparkFlex = mMotor->getSparkFlex()) {
      rev::spark::SparkFlexConfig config{};
      config.closedLoop.Pid(mConfig.kP, mConfig.kI, mConfig.kD);
      config.closedLoop.feedForward.kS(mConfig.kS).kV(mConfig.kV).kG(mConfig.kG);
      config.softLimit.ForwardSoftLimit(mConfig.kMaxHeight.value() / mConfig.kMetersPerRotation)
          .ForwardSoftLimitEnabled(true);
      config.softLimit.ReverseSoftLimit(mConfig.kMinHeight.value() / mConfig.kMetersPerRotation)
          .ReverseSoftLimitEnabled(true);
      config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
      sparkFlex->Configure(config,
                           rev::ResetMode::kNoResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
    }
#endif
  }

  StanMotor* mMotor{nullptr};
  bool mOwnsMotor{false};
  ElevatorConfig mConfig;
  units::length::meter_t mTargetHeight{0_m};
};

} // namespace stan
