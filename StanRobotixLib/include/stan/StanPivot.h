#pragma once

#include <algorithm>
#include <cmath>
#include <string_view>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/math.h>
#include "KrakenSync.h"
#include "StanMotor.h"

namespace stan {

struct PivotConfig {
  units::angle::degree_t kMinAngle{0_deg};
  units::angle::degree_t kMaxAngle{120_deg};
  units::angle::degree_t kTolerance{1.0_deg};
  double kGearRatio{50.0};
  double kP{40.0};
  double kI{0.0};
  double kD{0.5};
  double kS{0.0};
  double kG{0.3};
  double kV{0.0};
  bool kContinuousWrap{false};
};

class StanPivot : public frc2::SubsystemBase {
 public:
  StanPivot(StanMotor* iMotor, ctre::phoenix6::hardware::CANcoder* iEncoder, const PivotConfig& iConfig)
      : mMotor{iMotor}, mEncoder{iEncoder}, mOwnsMotor{false}, mOwnsEncoder{false}, mConfig{iConfig} {
    configureMotor();
    syncToAbsoluteEncoder();
  }

  StanPivot(int iCanId, MotorType iType, int iEncoderId, const PivotConfig& iConfig, std::string_view iCanBus = "rio")
      : mMotor{new StanMotor{iCanId, iType, iCanBus}},
        mEncoder{new ctre::phoenix6::hardware::CANcoder{iEncoderId, iCanBus}},
        mOwnsMotor{true},
        mOwnsEncoder{true},
        mConfig{iConfig} {
    configureMotor();
    syncToAbsoluteEncoder();
  }

  ~StanPivot() override {
    if (mOwnsMotor) {
      delete mMotor;
    }
    if (mOwnsEncoder) {
      delete mEncoder;
    }
  }

  void setTargetAngle(units::angle::degree_t iAngle) {
    units::angle::degree_t target = std::clamp(iAngle, mConfig.kMinAngle, mConfig.kMaxAngle);
    mTargetAngle = target;
    mMotor->setPosition(units::angle::turn_t{target});
  }

  void stop() {
    mMotor->stopMotor();
  }

  bool atTargetAngle() const {
    return units::math::abs(getAngle() - mTargetAngle) <= mConfig.kTolerance;
  }

  units::angle::degree_t getAngle() const {
    return units::angle::degree_t{mMotor->getPosition()};
  }

  units::angle::degree_t getTargetAngle() const {
    return mTargetAngle;
  }

  void setTolerance(units::angle::degree_t iTolerance) {
    mConfig.kTolerance = iTolerance;
  }

  void syncToAbsoluteEncoder() {
    if (mEncoder != nullptr) {
      if (auto* talon = mMotor->getTalonFX()) {
        KrakenSync::sync(talon, mEncoder, 250_ms);
      }
#if STAN_HAS_REV
      else if (auto* encoder = mMotor->getSparkRelativeEncoder()) {
        auto& posSignal = mEncoder->GetPosition();
        posSignal.WaitForUpdate(250_ms);
        if (posSignal.GetStatus().IsOK()) {
          encoder->SetPosition(posSignal.GetValue().value());
        }
      }
#endif
    }
  }

  StanMotor* getMotor() {
    return mMotor;
  }

  ctre::phoenix6::hardware::CANcoder* getEncoder() {
    return mEncoder;
  }

  frc2::CommandPtr goToAngle(units::angle::degree_t iAngle) {
    return Run([this, iAngle] { setTargetAngle(iAngle); });
  }

  frc2::CommandPtr holdAngle() {
    return Run([this] { setTargetAngle(mTargetAngle); });
  }

  frc2::CommandPtr syncEncoderCommand() {
    return RunOnce([this] { syncToAbsoluteEncoder(); });
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
      config.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
      config.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
      config.Feedback.SensorToMechanismRatio = mConfig.kGearRatio;
      config.ClosedLoopGeneral.ContinuousWrap = mConfig.kContinuousWrap;
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::angle::turn_t{mConfig.kMaxAngle};
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::angle::turn_t{mConfig.kMinAngle};
      config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
      talon->GetConfigurator().Apply(config);
    }
#if STAN_HAS_REV
    else if (auto* spark = mMotor->getSparkMax()) {
      rev::spark::SparkMaxConfig config{};
      config.closedLoop.Pid(mConfig.kP, mConfig.kI, mConfig.kD);
      config.closedLoop.feedForward.kS(mConfig.kS).kV(mConfig.kV).kCos(mConfig.kG);
      config.softLimit.ForwardSoftLimit(units::angle::turn_t{mConfig.kMaxAngle}.value())
          .ForwardSoftLimitEnabled(true);
      config.softLimit.ReverseSoftLimit(units::angle::turn_t{mConfig.kMinAngle}.value())
          .ReverseSoftLimitEnabled(true);
      config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
      spark->Configure(config,
                       rev::ResetMode::kNoResetSafeParameters,
                       rev::PersistMode::kPersistParameters);
    } else if (auto* sparkFlex = mMotor->getSparkFlex()) {
      rev::spark::SparkFlexConfig config{};
      config.closedLoop.Pid(mConfig.kP, mConfig.kI, mConfig.kD);
      config.closedLoop.feedForward.kS(mConfig.kS).kV(mConfig.kV).kCos(mConfig.kG);
      config.softLimit.ForwardSoftLimit(units::angle::turn_t{mConfig.kMaxAngle}.value())
          .ForwardSoftLimitEnabled(true);
      config.softLimit.ReverseSoftLimit(units::angle::turn_t{mConfig.kMinAngle}.value())
          .ReverseSoftLimitEnabled(true);
      config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
      sparkFlex->Configure(config,
                           rev::ResetMode::kNoResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
    }
#endif
  }

  StanMotor* mMotor{nullptr};
  ctre::phoenix6::hardware::CANcoder* mEncoder{nullptr};
  bool mOwnsMotor{false};
  bool mOwnsEncoder{false};
  PivotConfig mConfig;
  units::angle::degree_t mTargetAngle{0_deg};
};

} // namespace stan
