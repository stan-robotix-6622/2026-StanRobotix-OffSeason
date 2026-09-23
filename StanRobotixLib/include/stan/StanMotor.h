#pragma once

#include <algorithm>
#include <string_view>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/voltage.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/CurrentLimitsConfigs.hpp>
#include <ctre/phoenix6/configs/MotorOutputConfigs.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

#if __has_include(<rev/SparkMax.h>)
#define STAN_HAS_REV 1
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/config/SparkFlexConfig.h>
#include <rev/config/SparkMaxConfig.h>
#else
#define STAN_HAS_REV 0
namespace rev::spark {
class SparkMax;
class SparkFlex;
class SparkClosedLoopController;
class SparkRelativeEncoder;
} // namespace rev::spark
#endif

namespace stan {

enum class MotorType {
  kTalonFX,
  kSparkMax,
  kSparkFlex
};

enum class IdleMode {
  kBrake,
  kCoast
};

class StanMotor {
 public:
  StanMotor(int iCanId, MotorType iType, std::string_view iCanBus = "rio")
      : mType{iType}, mCanId{iCanId} {
    if (mType == MotorType::kTalonFX) {
      mTalonFX = new ctre::phoenix6::hardware::TalonFX{iCanId, iCanBus};
      ctre::phoenix6::configs::TalonFXConfiguration config{};
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 40_A;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.StatorCurrentLimit = 60_A;
      config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
      mTalonFX->GetConfigurator().Apply(config);
    }
#if STAN_HAS_REV
    else if (mType == MotorType::kSparkMax) {
      mSparkMax = new rev::spark::SparkMax{iCanId, rev::spark::SparkLowLevel::MotorType::kBrushless};
      rev::spark::SparkMaxConfig config{};
      config.SmartCurrentLimit(40);
      config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
      mSparkMax->Configure(config,
                           rev::ResetMode::kResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
      mSparkClosedLoop = new rev::spark::SparkClosedLoopController{mSparkMax->GetClosedLoopController()};
      mSparkEncoder = new rev::spark::SparkRelativeEncoder{mSparkMax->GetEncoder()};
    } else if (mType == MotorType::kSparkFlex) {
      mSparkFlex = new rev::spark::SparkFlex{iCanId, rev::spark::SparkLowLevel::MotorType::kBrushless};
      rev::spark::SparkFlexConfig config{};
      config.SmartCurrentLimit(40);
      config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
      mSparkFlex->Configure(config,
                            rev::ResetMode::kResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
      mSparkClosedLoop = new rev::spark::SparkClosedLoopController{mSparkFlex->GetClosedLoopController()};
      mSparkEncoder = new rev::spark::SparkRelativeEncoder{mSparkFlex->GetEncoder()};
    }
#endif
  }

  ~StanMotor() {
    delete mTalonFX;
#if STAN_HAS_REV
    delete mSparkMax;
    delete mSparkFlex;
    delete mSparkClosedLoop;
    delete mSparkEncoder;
#endif
  }

  void set(double iSpeed) {
    double speed = std::clamp(iSpeed, -1.0, 1.0);
    if (mTalonFX != nullptr) {
      mTalonFX->SetControl(mDutyCycleControl.WithOutput(speed));
    }
#if STAN_HAS_REV
    else if (mSparkMax != nullptr) {
      mSparkMax->Set(speed);
    } else if (mSparkFlex != nullptr) {
      mSparkFlex->Set(speed);
    }
#endif
  }

  void setVoltage(units::voltage::volt_t iVoltage) {
    if (mTalonFX != nullptr) {
      mTalonFX->SetControl(mVoltageControl.WithOutput(iVoltage));
    }
#if STAN_HAS_REV
    else if (mSparkMax != nullptr) {
      mSparkMax->SetVoltage(iVoltage);
    } else if (mSparkFlex != nullptr) {
      mSparkFlex->SetVoltage(iVoltage);
    }
#endif
  }

  void setVelocity(units::angular_velocity::turns_per_second_t iVelocity) {
    if (mTalonFX != nullptr) {
      mTalonFX->SetControl(mVelocityControl.WithVelocity(iVelocity));
    }
#if STAN_HAS_REV
    else if (mSparkClosedLoop != nullptr) {
      double rpm = iVelocity.value() * 60.0;
      mSparkClosedLoop->SetSetpoint(rpm, rev::spark::SparkLowLevel::ControlType::kVelocity);
    }
#endif
  }

  void setPosition(units::angle::turn_t iPosition) {
    if (mTalonFX != nullptr) {
      mTalonFX->SetControl(mPositionControl.WithPosition(iPosition));
    }
#if STAN_HAS_REV
    else if (mSparkClosedLoop != nullptr) {
      mSparkClosedLoop->SetSetpoint(iPosition.value(), rev::spark::SparkLowLevel::ControlType::kPosition);
    }
#endif
  }

  void stopMotor() {
    if (mTalonFX != nullptr) {
      mTalonFX->StopMotor();
    }
#if STAN_HAS_REV
    else if (mSparkMax != nullptr) {
      mSparkMax->StopMotor();
    } else if (mSparkFlex != nullptr) {
      mSparkFlex->StopMotor();
    }
#endif
  }

  void setIdleMode(IdleMode iMode) {
    if (mTalonFX != nullptr) {
      ctre::phoenix6::configs::MotorOutputConfigs config{};
      mTalonFX->GetConfigurator().Refresh(config);
      config.NeutralMode = (iMode == IdleMode::kBrake)
                               ? ctre::phoenix6::signals::NeutralModeValue::Brake
                               : ctre::phoenix6::signals::NeutralModeValue::Coast;
      mTalonFX->GetConfigurator().Apply(config);
    }
#if STAN_HAS_REV
    else if (mSparkMax != nullptr) {
      rev::spark::SparkMaxConfig config{};
      config.SetIdleMode((iMode == IdleMode::kBrake)
                             ? rev::spark::SparkBaseConfig::IdleMode::kBrake
                             : rev::spark::SparkBaseConfig::IdleMode::kCoast);
      mSparkMax->Configure(config,
                           rev::ResetMode::kNoResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
    } else if (mSparkFlex != nullptr) {
      rev::spark::SparkFlexConfig config{};
      config.SetIdleMode((iMode == IdleMode::kBrake)
                             ? rev::spark::SparkBaseConfig::IdleMode::kBrake
                             : rev::spark::SparkBaseConfig::IdleMode::kCoast);
      mSparkFlex->Configure(config,
                            rev::ResetMode::kNoResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    }
#endif
  }

  void setInverted(bool iInverted) {
    if (mTalonFX != nullptr) {
      ctre::phoenix6::configs::MotorOutputConfigs config{};
      mTalonFX->GetConfigurator().Refresh(config);
      config.Inverted = iInverted
                            ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
                            : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
      mTalonFX->GetConfigurator().Apply(config);
    }
#if STAN_HAS_REV
    else if (mSparkMax != nullptr) {
      rev::spark::SparkMaxConfig config{};
      config.Inverted(iInverted);
      mSparkMax->Configure(config,
                           rev::ResetMode::kNoResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
    } else if (mSparkFlex != nullptr) {
      rev::spark::SparkFlexConfig config{};
      config.Inverted(iInverted);
      mSparkFlex->Configure(config,
                            rev::ResetMode::kNoResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    }
#endif
  }

  void setCurrentLimit(units::current::ampere_t iLimit) {
    if (mTalonFX != nullptr) {
      ctre::phoenix6::configs::CurrentLimitsConfigs config{};
      mTalonFX->GetConfigurator().Refresh(config);
      config.SupplyCurrentLimitEnable = true;
      config.SupplyCurrentLimit = iLimit;
      mTalonFX->GetConfigurator().Apply(config);
    }
#if STAN_HAS_REV
    else if (mSparkMax != nullptr) {
      rev::spark::SparkMaxConfig config{};
      config.SmartCurrentLimit(static_cast<unsigned int>(iLimit.value()));
      mSparkMax->Configure(config,
                           rev::ResetMode::kNoResetSafeParameters,
                           rev::PersistMode::kPersistParameters);
    } else if (mSparkFlex != nullptr) {
      rev::spark::SparkFlexConfig config{};
      config.SmartCurrentLimit(static_cast<unsigned int>(iLimit.value()));
      mSparkFlex->Configure(config,
                            rev::ResetMode::kNoResetSafeParameters,
                            rev::PersistMode::kPersistParameters);
    }
#endif
  }

  units::voltage::volt_t getVoltage() const {
    if (mTalonFX != nullptr) {
      return mTalonFX->GetMotorVoltage().GetValue();
    }
#if STAN_HAS_REV
    else if (mSparkMax != nullptr) {
      return units::voltage::volt_t{mSparkMax->GetBusVoltage() * mSparkMax->GetAppliedOutput()};
    } else if (mSparkFlex != nullptr) {
      return units::voltage::volt_t{mSparkFlex->GetBusVoltage() * mSparkFlex->GetAppliedOutput()};
    }
#endif
    return 0_V;
  }

  units::current::ampere_t getCurrent() const {
    if (mTalonFX != nullptr) {
      return mTalonFX->GetStatorCurrent().GetValue();
    }
#if STAN_HAS_REV
    else if (mSparkMax != nullptr) {
      return units::current::ampere_t{mSparkMax->GetOutputCurrent()};
    } else if (mSparkFlex != nullptr) {
      return units::current::ampere_t{mSparkFlex->GetOutputCurrent()};
    }
#endif
    return 0_A;
  }

  units::angular_velocity::turns_per_second_t getVelocity() const {
    if (mTalonFX != nullptr) {
      return mTalonFX->GetVelocity().GetValue();
    }
#if STAN_HAS_REV
    else if (mSparkEncoder != nullptr) {
      return units::angular_velocity::turns_per_second_t{mSparkEncoder->GetVelocity() / 60.0};
    }
#endif
    return 0_tps;
  }

  units::angle::turn_t getPosition() const {
    if (mTalonFX != nullptr) {
      return mTalonFX->GetPosition().GetValue();
    }
#if STAN_HAS_REV
    else if (mSparkEncoder != nullptr) {
      return units::angle::turn_t{mSparkEncoder->GetPosition()};
    }
#endif
    return 0_tr;
  }

  ctre::phoenix6::hardware::TalonFX* getTalonFX() {
    return mTalonFX;
  }

  rev::spark::SparkMax* getSparkMax() {
#if STAN_HAS_REV
    return mSparkMax;
#else
    return nullptr;
#endif
  }

  rev::spark::SparkFlex* getSparkFlex() {
#if STAN_HAS_REV
    return mSparkFlex;
#else
    return nullptr;
#endif
  }

  rev::spark::SparkClosedLoopController* getSparkClosedLoopController() {
#if STAN_HAS_REV
    return mSparkClosedLoop;
#else
    return nullptr;
#endif
  }

  rev::spark::SparkRelativeEncoder* getSparkRelativeEncoder() {
#if STAN_HAS_REV
    return mSparkEncoder;
#else
    return nullptr;
#endif
  }

  MotorType getType() const {
    return mType;
  }

  int getCanId() const {
    return mCanId;
  }

 private:
  MotorType mType;
  int mCanId;

  ctre::phoenix6::hardware::TalonFX* mTalonFX{nullptr};
#if STAN_HAS_REV
  rev::spark::SparkMax* mSparkMax{nullptr};
  rev::spark::SparkFlex* mSparkFlex{nullptr};
  rev::spark::SparkClosedLoopController* mSparkClosedLoop{nullptr};
  rev::spark::SparkRelativeEncoder* mSparkEncoder{nullptr};
#endif

  ctre::phoenix6::controls::DutyCycleOut mDutyCycleControl{0.0};
  ctre::phoenix6::controls::VoltageOut mVoltageControl{0_V};
  ctre::phoenix6::controls::VelocityVoltage mVelocityControl{0_tps};
  ctre::phoenix6::controls::PositionVoltage mPositionControl{0_tr};
};

} // namespace stan
