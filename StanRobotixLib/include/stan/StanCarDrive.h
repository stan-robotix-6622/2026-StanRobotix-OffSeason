#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <string_view>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/MagnetSensorConfigs.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angle.h>
#include <units/current.h>
#include <units/time.h>
#include <units/velocity.h>

namespace stan {

enum class CarDriveLayout {
  kFrontSteerFrontDrive,
  kFrontSteerRearDrive
};

struct StanCarDriveConfig {
  int kFLDriveId{1};
  int kFRDriveId{2};
  int kFLSteerId{3};
  int kFRSteerId{4};
  int kFLEncoderId{5};
  int kFREncoderId{6};
  std::string_view kCanBus{"rio"};

  double kSteerGearRatio{12.8};
  units::angle::degree_t kMaxSteerAngle{60.0_deg};
  double kSpeedScale{0.5};

  units::angle::turn_t kFLMagnetOffset{0.0_tr};
  units::angle::turn_t kFRMagnetOffset{0.0_tr};

  bool kFLDriveInverted{false};
  bool kFRDriveInverted{false};
  bool kFLSteerInverted{false};
  bool kFRSteerInverted{false};

  double kSteerP{40.0};
  double kSteerI{0.0};
  double kSteerD{0.5};
  double kSteerS{0.0};

  units::current::ampere_t kSupplyCurrentLimit{40.0_A};
  units::current::ampere_t kStatorCurrentLimit{60.0_A};

  CarDriveLayout kLayout{CarDriveLayout::kFrontSteerFrontDrive};
};

class StanCarDrive : public frc2::SubsystemBase {
 public:
  explicit StanCarDrive(const StanCarDriveConfig& iConfig = StanCarDriveConfig{})
      : mConfig{iConfig},
        mSteerGearRatio{iConfig.kSteerGearRatio},
        mMaxSteerAngle{iConfig.kMaxSteerAngle},
        mSpeedScale{iConfig.kSpeedScale},
        mFLMagnetOffset{iConfig.kFLMagnetOffset.value()},
        mFRMagnetOffset{iConfig.kFRMagnetOffset.value()},
        mFLDriveInverted{iConfig.kFLDriveInverted},
        mFRDriveInverted{iConfig.kFRDriveInverted},
        mFLSteerInverted{iConfig.kFLSteerInverted},
        mFRSteerInverted{iConfig.kFRSteerInverted},
        mP{iConfig.kSteerP},
        mI{iConfig.kSteerI},
        mD{iConfig.kSteerD},
        mS{iConfig.kSteerS},
        mLayout{iConfig.kLayout} {
    mFLDrive = new ctre::phoenix6::hardware::TalonFX{iConfig.kFLDriveId, iConfig.kCanBus};
    mFRDrive = new ctre::phoenix6::hardware::TalonFX{iConfig.kFRDriveId, iConfig.kCanBus};
    mFLSteer = new ctre::phoenix6::hardware::TalonFX{iConfig.kFLSteerId, iConfig.kCanBus};
    mFRSteer = new ctre::phoenix6::hardware::TalonFX{iConfig.kFRSteerId, iConfig.kCanBus};
    mFLEncoder = new ctre::phoenix6::hardware::CANcoder{iConfig.kFLEncoderId, iConfig.kCanBus};
    mFREncoder = new ctre::phoenix6::hardware::CANcoder{iConfig.kFREncoderId, iConfig.kCanBus};

    ctre::phoenix6::configs::CANcoderConfiguration cancoderConfig{};
    cancoderConfig.MagnetSensor.MagnetOffset = units::angle::turn_t{mFLMagnetOffset};
    cancoderConfig.MagnetSensor.SensorDirection =
        ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFLEncoder->GetConfigurator().Apply(cancoderConfig);

    cancoderConfig.MagnetSensor.MagnetOffset = units::angle::turn_t{mFRMagnetOffset};
    mFREncoder->GetConfigurator().Apply(cancoderConfig);

    ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = iConfig.kSupplyCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = iConfig.kStatorCurrentLimit;
    driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    driveConfig.MotorOutput.Inverted = mFLDriveInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    mFLDrive->GetConfigurator().Apply(driveConfig);

    driveConfig.MotorOutput.Inverted = mFRDriveInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    mFRDrive->GetConfigurator().Apply(driveConfig);

    ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerConfig.CurrentLimits.SupplyCurrentLimit = iConfig.kSupplyCurrentLimit;
    steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    steerConfig.CurrentLimits.StatorCurrentLimit = iConfig.kStatorCurrentLimit;
    steerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    steerConfig.MotorOutput.Inverted = mFLSteerInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    steerConfig.Feedback.FeedbackSensorSource =
        ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    steerConfig.Feedback.SensorToMechanismRatio = mSteerGearRatio;
    steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
    steerConfig.Slot0.kP = mP;
    steerConfig.Slot0.kI = mI;
    steerConfig.Slot0.kD = mD;
    steerConfig.Slot0.kS = mS;
    mFLSteer->GetConfigurator().Apply(steerConfig);

    steerConfig.MotorOutput.Inverted = mFRSteerInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    mFRSteer->GetConfigurator().Apply(steerConfig);

    syncSteerToCANcoder();
    initDashboard();
  }

  StanCarDrive(int iFLDriveId, int iFRDriveId, int iFLSteerId, int iFRSteerId,
               int iFLEncoderId, int iFREncoderId,
               units::angle::degree_t iMaxSteerAngle = 60.0_deg,
               double iSpeedScale = 0.5,
               std::string_view iCanBus = "rio")
      : StanCarDrive([&] {
          StanCarDriveConfig config;
          config.kFLDriveId = iFLDriveId;
          config.kFRDriveId = iFRDriveId;
          config.kFLSteerId = iFLSteerId;
          config.kFRSteerId = iFRSteerId;
          config.kFLEncoderId = iFLEncoderId;
          config.kFREncoderId = iFREncoderId;
          config.kMaxSteerAngle = iMaxSteerAngle;
          config.kSpeedScale = iSpeedScale;
          config.kCanBus = iCanBus;
          return config;
        }()) {}

  ~StanCarDrive() override {
    delete mFLDrive;
    delete mFRDrive;
    delete mFLSteer;
    delete mFRSteer;
    delete mFLEncoder;
    delete mFREncoder;
  }

  void Periodic() override {
    updateConfigsFromDashboard();
    updateTelemetry();
  }

  void drive(double iThrottle, double iBrake, double iSteer, bool iReverse = false) {
    if (frc::SmartDashboard::GetBoolean("Steer/TestEnable", false)) {
      return;
    }

    double throttle = std::clamp(iThrottle, 0.0, 1.0);
    double brake = std::clamp(iBrake, 0.0, 1.0);
    double speed = (throttle * mSpeedScale) * (1.0 - brake);
    if (speed < 0.0) {
      speed = 0.0;
    }
    if (iReverse) {
      speed = -speed;
    }

    units::angle::degree_t steerAngle = -std::clamp(iSteer, -1.0, 1.0) * mMaxSteerAngle;
    mTargetSteerAngle = steerAngle;

    mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(speed));
    mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(speed));

    mFLSteer->SetControl(mSteerPositionControl.WithPosition(steerAngle));
    mFRSteer->SetControl(mSteerPositionControl.WithPosition(steerAngle));
  }

  void stop() {
    mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
    mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
  }

  void stopDrive() {
    stop();
  }

  void setSteerAngle(units::angle::degree_t iAngle) {
    units::angle::degree_t clampedAngle = std::clamp(iAngle, -mMaxSteerAngle, mMaxSteerAngle);
    mTargetSteerAngle = clampedAngle;
    mFLSteer->SetControl(mSteerPositionControl.WithPosition(clampedAngle));
    mFRSteer->SetControl(mSteerPositionControl.WithPosition(clampedAngle));
  }

  void syncSteerToCANcoder() {
    mFLEncoder->GetPosition().WaitForUpdate(250_ms);
    mFREncoder->GetPosition().WaitForUpdate(250_ms);
    mFLSteer->SetPosition(mFLEncoder->GetPosition().GetValue());
    mFRSteer->SetPosition(mFREncoder->GetPosition().GetValue());
  }

  void zeroFL() {
    double currentPos = mFLEncoder->GetAbsolutePosition().GetValue().value();
    mFLMagnetOffset -= currentPos;
    while (mFLMagnetOffset > 1.0) mFLMagnetOffset -= 1.0;
    while (mFLMagnetOffset < -1.0) mFLMagnetOffset += 1.0;

    ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
    magConfig.MagnetOffset = units::angle::turn_t{mFLMagnetOffset};
    magConfig.SensorDirection =
        ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFLEncoder->GetConfigurator().Apply(magConfig);
    mFLEncoder->GetPosition().WaitForUpdate(250_ms);
    mFLSteer->SetPosition(mFLEncoder->GetPosition().GetValue());

    frc::SmartDashboard::PutNumber("Steer/FLMagnetOffset", mFLMagnetOffset);
  }

  void zeroFR() {
    double currentPos = mFREncoder->GetAbsolutePosition().GetValue().value();
    mFRMagnetOffset -= currentPos;
    while (mFRMagnetOffset > 1.0) mFRMagnetOffset -= 1.0;
    while (mFRMagnetOffset < -1.0) mFRMagnetOffset += 1.0;

    ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
    magConfig.MagnetOffset = units::angle::turn_t{mFRMagnetOffset};
    magConfig.SensorDirection =
        ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFREncoder->GetConfigurator().Apply(magConfig);
    mFREncoder->GetPosition().WaitForUpdate(250_ms);
    mFRSteer->SetPosition(mFREncoder->GetPosition().GetValue());

    frc::SmartDashboard::PutNumber("Steer/FRMagnetOffset", mFRMagnetOffset);
  }

  units::angle::degree_t getSteerAngle() const {
    return mTargetSteerAngle;
  }

  units::angle::degree_t getMaxSteerAngle() const {
    return mMaxSteerAngle;
  }

  double getSpeedScale() const {
    return mSpeedScale;
  }

  void setSpeedScale(double iScale) {
    mSpeedScale = std::clamp(iScale, 0.0, 1.0);
  }

  CarDriveLayout getLayout() const {
    return mLayout;
  }

  frc2::CommandPtr getZeroFLCommand() {
    return RunOnce([this] { zeroFL(); });
  }

  frc2::CommandPtr getZeroFRCommand() {
    return RunOnce([this] { zeroFR(); });
  }

  frc2::CommandPtr getTestSteerCommand() {
    return Run([this] {
      double targetAngleDeg = frc::SmartDashboard::GetNumber("Steer/TestTargetAngleDeg", 45.0);
      setSteerAngle(units::angle::degree_t{targetAngleDeg});
      stopDrive();
    });
  }

  frc2::CommandPtr getDriveCommand(
      std::function<double()> iThrottleSupplier,
      std::function<double()> iBrakeSupplier,
      std::function<double()> iSteerSupplier,
      std::function<bool()> iReverseSupplier = [] { return false; }) {
    return Run([this, iThrottleSupplier, iBrakeSupplier, iSteerSupplier, iReverseSupplier] {
      drive(iThrottleSupplier(), iBrakeSupplier(), iSteerSupplier(), iReverseSupplier());
    });
  }

  frc2::CommandPtr getStopCommand() {
    return RunOnce([this] { stop(); });
  }

  ctre::phoenix6::hardware::TalonFX* getFLDrive() { return mFLDrive; }
  ctre::phoenix6::hardware::TalonFX* getFRDrive() { return mFRDrive; }
  ctre::phoenix6::hardware::TalonFX* getFLSteer() { return mFLSteer; }
  ctre::phoenix6::hardware::TalonFX* getFRSteer() { return mFRSteer; }
  ctre::phoenix6::hardware::CANcoder* getFLEncoder() { return mFLEncoder; }
  ctre::phoenix6::hardware::CANcoder* getFREncoder() { return mFREncoder; }

 private:
  void initDashboard() {
    frc::SmartDashboard::SetDefaultBoolean("Steer/SyncCANcoder", false);
    frc::SmartDashboard::SetDefaultNumber("Steer/kP", mP);
    frc::SmartDashboard::SetDefaultNumber("Steer/kI", mI);
    frc::SmartDashboard::SetDefaultNumber("Steer/kD", mD);
    frc::SmartDashboard::SetDefaultNumber("Steer/kS", mS);

    frc::SmartDashboard::SetDefaultNumber("Steer/FLMagnetOffset", mFLMagnetOffset);
    frc::SmartDashboard::SetDefaultNumber("Steer/FRMagnetOffset", mFRMagnetOffset);

    frc::SmartDashboard::SetDefaultBoolean("Drive/FLDriveInverted", mFLDriveInverted);
    frc::SmartDashboard::SetDefaultBoolean("Drive/FRDriveInverted", mFRDriveInverted);
    frc::SmartDashboard::SetDefaultBoolean("Steer/FLSteerInverted", mFLSteerInverted);
    frc::SmartDashboard::SetDefaultBoolean("Steer/FRSteerInverted", mFRSteerInverted);

    frc::SmartDashboard::SetDefaultNumber("Steer/GearRatio", mSteerGearRatio);
    frc::SmartDashboard::SetDefaultNumber("Steer/MaxAngleDeg", mMaxSteerAngle.value());
    frc::SmartDashboard::SetDefaultNumber("Drive/SpeedScale", mSpeedScale);

    frc::SmartDashboard::SetDefaultNumber("Steer/TestTargetAngleDeg", 45.0);
    frc::SmartDashboard::SetDefaultBoolean("Steer/TestEnable", false);
    frc::SmartDashboard::SetDefaultBoolean("Steer/ZeroFL", false);
    frc::SmartDashboard::SetDefaultBoolean("Steer/ZeroFR", false);
  }

  void updateConfigsFromDashboard() {
    double newSpeedScale = frc::SmartDashboard::GetNumber("Drive/SpeedScale", mSpeedScale);
    if (std::abs(newSpeedScale - mSpeedScale) > 1e-6) {
      mSpeedScale = newSpeedScale;
    }

    double newMaxSteerAngleDeg = frc::SmartDashboard::GetNumber("Steer/MaxAngleDeg", mMaxSteerAngle.value());
    if (std::abs(newMaxSteerAngleDeg - mMaxSteerAngle.value()) > 1e-6) {
      mMaxSteerAngle = units::angle::degree_t{newMaxSteerAngleDeg};
    }

    if (!frc::DriverStation::IsTest()) {
      return;
    }

    double newP = frc::SmartDashboard::GetNumber("Steer/kP", mP);
    double newI = frc::SmartDashboard::GetNumber("Steer/kI", mI);
    double newD = frc::SmartDashboard::GetNumber("Steer/kD", mD);
    double newS = frc::SmartDashboard::GetNumber("Steer/kS", mS);

    if (std::abs(newP - mP) > 1e-6 || std::abs(newI - mI) > 1e-6 ||
        std::abs(newD - mD) > 1e-6 || std::abs(newS - mS) > 1e-6) {
      mP = newP;
      mI = newI;
      mD = newD;
      mS = newS;

      ctre::phoenix6::configs::Slot0Configs slot0Config{};
      slot0Config.kP = mP;
      slot0Config.kI = mI;
      slot0Config.kD = mD;
      slot0Config.kS = mS;
      mFLSteer->GetConfigurator().Apply(slot0Config);
      mFRSteer->GetConfigurator().Apply(slot0Config);
    }

    if (frc::SmartDashboard::GetBoolean("Steer/ZeroFL", false)) {
      zeroFL();
      frc::SmartDashboard::PutBoolean("Steer/ZeroFL", false);
    }

    if (frc::SmartDashboard::GetBoolean("Steer/ZeroFR", false)) {
      zeroFR();
      frc::SmartDashboard::PutBoolean("Steer/ZeroFR", false);
    }
  }

  void updateTelemetry() {
    double targetDeg = mTargetSteerAngle.value();
    double flDeg = units::angle::degree_t{mFLSteer->GetPosition().GetValue()}.value();
    double frDeg = units::angle::degree_t{mFRSteer->GetPosition().GetValue()}.value();

    frc::SmartDashboard::PutNumber("Steer/TargetAngleDeg", targetDeg);
    frc::SmartDashboard::PutNumber("Steer/FLAngleDeg", flDeg);
    frc::SmartDashboard::PutNumber("Steer/FRAngleDeg", frDeg);
    frc::SmartDashboard::PutNumber("Steer/FLErrDeg", targetDeg - flDeg);
    frc::SmartDashboard::PutNumber("Steer/FRErrDeg", targetDeg - frDeg);

    frc::SmartDashboard::PutNumber("Steer/FLEncoderDeg",
                                   units::angle::degree_t{mFLEncoder->GetPosition().GetValue()}.value());
    frc::SmartDashboard::PutNumber("Steer/FREncoderDeg",
                                   units::angle::degree_t{mFREncoder->GetPosition().GetValue()}.value());

    frc::SmartDashboard::PutBoolean("Steer/FLCANcoderConnected", mFLEncoder->IsConnected());
    frc::SmartDashboard::PutBoolean("Steer/FRCANcoderConnected", mFREncoder->IsConnected());

    frc::SmartDashboard::PutNumber("Steer/FLCurrentAmps", mFLSteer->GetStatorCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Steer/FRCurrentAmps", mFRSteer->GetStatorCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Drive/FLCurrentAmps", mFLDrive->GetStatorCurrent().GetValue().value());
    frc::SmartDashboard::PutNumber("Drive/FRCurrentAmps", mFRDrive->GetStatorCurrent().GetValue().value());
  }

  StanCarDriveConfig mConfig;

  ctre::phoenix6::hardware::TalonFX* mFLDrive{nullptr};
  ctre::phoenix6::hardware::TalonFX* mFRDrive{nullptr};
  ctre::phoenix6::hardware::TalonFX* mFLSteer{nullptr};
  ctre::phoenix6::hardware::TalonFX* mFRSteer{nullptr};
  ctre::phoenix6::hardware::CANcoder* mFLEncoder{nullptr};
  ctre::phoenix6::hardware::CANcoder* mFREncoder{nullptr};

  ctre::phoenix6::controls::DutyCycleOut mDriveDutyCycleControl{0.0};
  ctre::phoenix6::controls::PositionVoltage mSteerPositionControl{0_tr};

  double mSteerGearRatio{12.8};
  units::angle::degree_t mMaxSteerAngle{60.0_deg};
  double mSpeedScale{0.5};

  double mFLMagnetOffset{0.0};
  double mFRMagnetOffset{0.0};

  bool mFLDriveInverted{false};
  bool mFRDriveInverted{false};
  bool mFLSteerInverted{false};
  bool mFRSteerInverted{false};

  double mP{40.0};
  double mI{0.0};
  double mD{0.5};
  double mS{0.0};

  units::angle::degree_t mTargetSteerAngle{0.0_deg};
  CarDriveLayout mLayout{CarDriveLayout::kFrontSteerFrontDrive};
};

} // namespace stan
