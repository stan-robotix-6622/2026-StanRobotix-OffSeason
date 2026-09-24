#include "subsystems/SubCarDrive.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <ctre/phoenix6/configs/FeedbackConfigs.hpp>
#include <ctre/phoenix6/configs/MagnetSensorConfigs.hpp>
#include <ctre/phoenix6/configs/MotorOutputConfigs.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include "Constants.h"

SubCarDrive::SubCarDrive() {
  mFLDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFLDrive};
  mFLSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFLSteer};
  mFLEncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFLEncoder};

  mFRDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFRDrive};
  mFRSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFRSteer};
  mFREncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFREncoder};

  mPigeon = new ctre::phoenix6::hardware::Pigeon2{CANid::kPigeon2};

  mP = CarDriveConstants::kSteerP;
  mI = CarDriveConstants::kSteerI;
  mD = CarDriveConstants::kSteerD;
  mS = CarDriveConstants::kSteerS;

  mFLMagnetOffset = CarDriveConstants::kFLMagnetOffset.value();
  mFRMagnetOffset = CarDriveConstants::kFRMagnetOffset.value();

  mFLDriveInverted = CarDriveConstants::kFLDriveInverted;
  mFRDriveInverted = CarDriveConstants::kFRDriveInverted;
  mFLSteerInverted = CarDriveConstants::kFLSteerInverted;
  mFRSteerInverted = CarDriveConstants::kFRSteerInverted;

  mSteerGearRatio = CarDriveConstants::kSteerGearRatio;
  mMaxSteerAngle = CarDriveConstants::kMaxSteerAngle;
  mSpeedScale = CarDriveConstants::kSpeedScale;
  mTrackWidth = CarDriveConstants::kTrackWidth;
  mWheelBase = CarDriveConstants::kWheelBase;
  mWheelRadius = CarDriveConstants::kWheelRadius;

  mDriveGearRatio = CarDriveConstants::kDriveGearRatio;
  mTractionSlipThreshold = CarDriveConstants::kTractionSlipThreshold;
  mTractionKp = CarDriveConstants::kTractionKp;
  mYawStabilityKp = CarDriveConstants::kYawStabilityKp;
  mDriftTorqueVectorScale = CarDriveConstants::kDriftTorqueVectorScale;

  ctre::phoenix6::configs::CANcoderConfiguration cancoderConfig{};
  cancoderConfig.MagnetSensor.MagnetOffset = units::angle::turn_t{mFLMagnetOffset};
  cancoderConfig.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  mFLEncoder->GetConfigurator().Apply(cancoderConfig);

  cancoderConfig.MagnetSensor.MagnetOffset = units::angle::turn_t{mFRMagnetOffset};
  mFREncoder->GetConfigurator().Apply(cancoderConfig);

  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
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
  steerConfig.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  steerConfig.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
  steerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.MotorOutput.Inverted = mFLSteerInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
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

SubCarDrive::~SubCarDrive() {
  delete mFLDrive;
  delete mFLSteer;
  delete mFLEncoder;

  delete mFRDrive;
  delete mFRSteer;
  delete mFREncoder;

  delete mPigeon;
}

void SubCarDrive::Periodic() {
  updateConfigsFromDashboard();
  updateTelemetry();
}

void SubCarDrive::drive(double iThrottle, double iBrake, double iSteer, bool iReverse, bool iDrift) {
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

  units::angle::radian_t delta{steerAngle};
  double L = mWheelBase.value();
  double halfW = (mTrackWidth / 2.0).value();

  double speedFL = speed;
  double speedFR = speed;
  units::angle::degree_t angleFL{0.0_deg};
  units::angle::degree_t angleFR{0.0_deg};

  if (std::abs(delta.value()) > 1e-4) {
    double tanDelta = std::tan(delta.value());
    double R = L / tanDelta;

    angleFL = units::angle::degree_t{std::atan(L / (R - halfW))};
    angleFR = units::angle::degree_t{std::atan(L / (R + halfW))};

    angleFL = std::clamp(angleFL, -mMaxSteerAngle, mMaxSteerAngle);
    angleFR = std::clamp(angleFR, -mMaxSteerAngle, mMaxSteerAngle);

    double dFL = std::hypot(L, R - halfW);
    double dFR = std::hypot(L, R + halfW);
    double dCenter = std::hypot(L, R);

    double maxRatio = std::max(dFL, dFR) / dCenter;
    speedFL = speed * (dFL / dCenter);
    speedFR = speed * (dFR / dCenter);

    if (maxRatio > 1.0 && std::abs(speed) > 1e-4) {
      speedFL /= maxRatio;
      speedFR /= maxRatio;
    }
  }

  double wheelCircumference = 2.0 * std::numbers::pi * units::meter_t{mWheelRadius}.value();
  double rpsFL = mFLDrive->GetVelocity().GetValue().value();
  double rpsFR = mFRDrive->GetVelocity().GetValue().value();
  double wheelSpeedFL = (rpsFL / mDriveGearRatio) * wheelCircumference;
  double wheelSpeedFR = (rpsFR / mDriveGearRatio) * wheelCircumference;
  double accelFL = (wheelSpeedFL - mPreviousWheelSpeedFL) / 0.02;
  double accelFR = (wheelSpeedFR - mPreviousWheelSpeedFR) / 0.02;
  mPreviousWheelSpeedFL = wheelSpeedFL;
  mPreviousWheelSpeedFR = wheelSpeedFR;

  units::acceleration::meters_per_second_squared_t imuAccelX = mPigeon->GetAccelerationX().GetValue();
  units::angular_velocity::degrees_per_second_t gyroYawRate = mPigeon->GetAngularVelocityZDevice().GetValue();

  mDriftActive = iDrift;
  if (iDrift) {
    mIsSlipping = false;
    if (steerAngle > 1.0_deg) {
      speedFR = std::clamp(speedFR * mDriftTorqueVectorScale, -1.0, 1.0);
      speedFL = speedFL * (1.0 - std::abs(iSteer) * 0.8);
    } else if (steerAngle < -1.0_deg) {
      speedFL = std::clamp(speedFL * mDriftTorqueVectorScale, -1.0, 1.0);
      speedFR = speedFR * (1.0 - std::abs(iSteer) * 0.8);
    }
  } else if (mTractionControlEnabled) {
    double forwardSign = (speed >= 0.0) ? 1.0 : -1.0;
    double measuredAx = imuAccelX.value() * forwardSign;
    double effectiveAccelFL = accelFL * forwardSign;
    double effectiveAccelFR = accelFR * forwardSign;

    double slipFL = effectiveAccelFL - measuredAx;
    double slipFR = effectiveAccelFR - measuredAx;

    bool slipFLDetected = (slipFL > mTractionSlipThreshold) && (std::abs(speed) > 0.05);
    bool slipFRDetected = (slipFR > mTractionSlipThreshold) && (std::abs(speed) > 0.05);
    mIsSlipping = slipFLDetected || slipFRDetected;

    if (slipFLDetected) {
      double reductionFL = (slipFL - mTractionSlipThreshold) * mTractionKp;
      speedFL *= std::clamp(1.0 - reductionFL, 0.05, 1.0);
    }
    if (slipFRDetected) {
      double reductionFR = (slipFR - mTractionSlipThreshold) * mTractionKp;
      speedFR *= std::clamp(1.0 - reductionFR, 0.05, 1.0);
    }

    double linearVelocity = (wheelSpeedFL + wheelSpeedFR) * 0.5;
    double wheelbaseMeters = units::meter_t{mWheelBase}.value();
    double desiredYawRateRadPerSec = (linearVelocity / wheelbaseMeters) * std::tan(delta.value());
    double desiredYawRateDps = desiredYawRateRadPerSec * 180.0 / std::numbers::pi;
    double yawRateError = desiredYawRateDps - gyroYawRate.value();
    double yawCorrection = yawRateError * mYawStabilityKp;

    speedFL = std::clamp(speedFL - yawCorrection, -1.0, 1.0);
    speedFR = std::clamp(speedFR + yawCorrection, -1.0, 1.0);
  } else {
    mIsSlipping = false;
  }

  mTargetSteerAngleFL = angleFL;
  mTargetSteerAngleFR = angleFR;
  mTargetSpeedFL = speedFL;
  mTargetSpeedFR = speedFR;

  mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(speedFL));
  mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(speedFR));

  mFLSteer->SetControl(mSteerPositionControl.WithPosition(angleFL));
  mFRSteer->SetControl(mSteerPositionControl.WithPosition(angleFR));
}

void SubCarDrive::stop() {
  mTargetSpeedFL = 0.0;
  mTargetSpeedFR = 0.0;
  mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
  mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
}

void SubCarDrive::stopDrive() {
  stop();
}

void SubCarDrive::setSteerAngle(units::angle::degree_t iAngle) {
  units::angle::degree_t clampedAngle = std::clamp(iAngle, -mMaxSteerAngle, mMaxSteerAngle);
  mTargetSteerAngle = clampedAngle;
  mTargetSteerAngleFL = clampedAngle;
  mTargetSteerAngleFR = clampedAngle;
  mFLSteer->SetControl(mSteerPositionControl.WithPosition(clampedAngle));
  mFRSteer->SetControl(mSteerPositionControl.WithPosition(clampedAngle));
}

void SubCarDrive::syncSteerToCANcoder() {
  mFLEncoder->GetPosition().WaitForUpdate(250_ms);
  mFREncoder->GetPosition().WaitForUpdate(250_ms);

  mFLSteer->SetPosition(mFLEncoder->GetPosition().GetValue());
  mFRSteer->SetPosition(mFREncoder->GetPosition().GetValue());
}

void SubCarDrive::zeroFL() {
  double currentPos = mFLEncoder->GetAbsolutePosition().GetValue().value();
  mFLMagnetOffset -= currentPos;
  while (mFLMagnetOffset > 1.0) mFLMagnetOffset -= 1.0;
  while (mFLMagnetOffset < -1.0) mFLMagnetOffset += 1.0;

  ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
  magConfig.MagnetOffset = units::angle::turn_t{mFLMagnetOffset};
  magConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  mFLEncoder->GetConfigurator().Apply(magConfig);
  mFLEncoder->GetPosition().WaitForUpdate(250_ms);
  mFLSteer->SetPosition(mFLEncoder->GetPosition().GetValue());

  frc::SmartDashboard::PutNumber("Steer/FLMagnetOffset", mFLMagnetOffset);
}

void SubCarDrive::zeroFR() {
  double currentPos = mFREncoder->GetAbsolutePosition().GetValue().value();
  mFRMagnetOffset -= currentPos;
  while (mFRMagnetOffset > 1.0) mFRMagnetOffset -= 1.0;
  while (mFRMagnetOffset < -1.0) mFRMagnetOffset += 1.0;

  ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
  magConfig.MagnetOffset = units::angle::turn_t{mFRMagnetOffset};
  magConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  mFREncoder->GetConfigurator().Apply(magConfig);
  mFREncoder->GetPosition().WaitForUpdate(250_ms);
  mFRSteer->SetPosition(mFREncoder->GetPosition().GetValue());

  frc::SmartDashboard::PutNumber("Steer/FRMagnetOffset", mFRMagnetOffset);
}

frc2::CommandPtr SubCarDrive::getZeroFLCommand() {
  return RunOnce([this] { zeroFL(); });
}

frc2::CommandPtr SubCarDrive::getZeroFRCommand() {
  return RunOnce([this] { zeroFR(); });
}

frc2::CommandPtr SubCarDrive::getTestSteerCommand() {
  return Run([this] {
    double targetAngleDeg = frc::SmartDashboard::GetNumber("Steer/TestTargetAngleDeg", 45.0);
    setSteerAngle(units::angle::degree_t{targetAngleDeg});
    stopDrive();
  });
}

frc2::CommandPtr SubCarDrive::getCalibrateRatioCommand() {
  struct CalibState {
    double startRotationsFL{0.0};
    double startRotationsFR{0.0};
    double integratedVelocity{0.0};
    double integratedDistance{0.0};
    double accelBias{0.0};
  };
  auto state = std::make_shared<CalibState>();

  return frc2::cmd::Sequence(
      RunOnce([this, state] {
        state->startRotationsFL = mFLDrive->GetPosition().GetValue().value();
        state->startRotationsFR = mFRDrive->GetPosition().GetValue().value();
        state->integratedVelocity = 0.0;
        state->integratedDistance = 0.0;
        units::acceleration::meters_per_second_squared_t a = mPigeon->GetAccelerationX().GetValue();
        state->accelBias = a.value();
      }),
      Run([this, state] {
        mFLSteer->SetControl(mSteerPositionControl.WithPosition(0.0_deg));
        mFRSteer->SetControl(mSteerPositionControl.WithPosition(0.0_deg));
        mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.18));
        mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.18));

        units::acceleration::meters_per_second_squared_t a = mPigeon->GetAccelerationX().GetValue();
        double ax = a.value() - state->accelBias;
        state->integratedVelocity += ax * 0.02;
        if (state->integratedVelocity < 0.0) {
          state->integratedVelocity = 0.0;
        }
        state->integratedDistance += state->integratedVelocity * 0.02;
      }).WithTimeout(1.2_s),
      RunOnce([this, state] {
        stop();
        double deltaFL = std::abs(mFLDrive->GetPosition().GetValue().value() - state->startRotationsFL);
        double deltaFR = std::abs(mFRDrive->GetPosition().GetValue().value() - state->startRotationsFR);
        double avgRotations = (deltaFL + deltaFR) * 0.5;
        double wheelCircumference = 2.0 * std::numbers::pi * units::meter_t{mWheelRadius}.value();

        if (state->integratedDistance > 0.05 && avgRotations > 0.1) {
          double estimatedRatio = (avgRotations * wheelCircumference) / state->integratedDistance;
          if (estimatedRatio > 1.0 && estimatedRatio < 25.0) {
            mDriveGearRatio = estimatedRatio;
            frc::SmartDashboard::PutNumber("Drive/DriveGearRatio", mDriveGearRatio);
          }
          frc::SmartDashboard::PutNumber("Drive/CalibratedGearRatio", estimatedRatio);
        }
      }));
}

void SubCarDrive::initDashboard() {
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

  frc::SmartDashboard::SetDefaultNumber("CarDrive/TrackWidthInches", mTrackWidth.value());
  frc::SmartDashboard::SetDefaultNumber("CarDrive/WheelBaseInches", mWheelBase.value());
  frc::SmartDashboard::SetDefaultNumber("Steer/TestTargetAngleDeg", 45.0);
  frc::SmartDashboard::SetDefaultBoolean("Steer/TestEnable", false);
  frc::SmartDashboard::SetDefaultBoolean("Steer/ZeroFL", false);
  frc::SmartDashboard::SetDefaultBoolean("Steer/ZeroFR", false);

  frc::SmartDashboard::SetDefaultBoolean("TractionControl/Enable", mTractionControlEnabled);
  frc::SmartDashboard::SetDefaultNumber("TractionControl/SlipThreshold", mTractionSlipThreshold);
  frc::SmartDashboard::SetDefaultNumber("TractionControl/Kp", mTractionKp);
  frc::SmartDashboard::SetDefaultNumber("TractionControl/YawKp", mYawStabilityKp);
  frc::SmartDashboard::SetDefaultNumber("Drift/TorqueVectorScale", mDriftTorqueVectorScale);
  frc::SmartDashboard::SetDefaultNumber("Drive/DriveGearRatio", mDriveGearRatio);
}

void SubCarDrive::updateConfigsFromDashboard() {
  double newTrackWidth = frc::SmartDashboard::GetNumber("CarDrive/TrackWidthInches", mTrackWidth.value());
  if (std::abs(newTrackWidth - mTrackWidth.value()) > 1e-6) {
    mTrackWidth = units::length::inch_t{newTrackWidth};
  }

  double newWheelBase = frc::SmartDashboard::GetNumber("CarDrive/WheelBaseInches", mWheelBase.value());
  if (std::abs(newWheelBase - mWheelBase.value()) > 1e-6) {
    mWheelBase = units::length::inch_t{newWheelBase};
  }

  double newSpeedScale = frc::SmartDashboard::GetNumber("Drive/SpeedScale", mSpeedScale);
  if (std::abs(newSpeedScale - mSpeedScale) > 1e-6) {
    mSpeedScale = newSpeedScale;
  }

  double newMaxSteerAngleDeg = frc::SmartDashboard::GetNumber("Steer/MaxAngleDeg", mMaxSteerAngle.value());
  if (std::abs(newMaxSteerAngleDeg - mMaxSteerAngle.value()) > 1e-6) {
    mMaxSteerAngle = units::angle::degree_t{newMaxSteerAngleDeg};
  }

  mTractionControlEnabled = frc::SmartDashboard::GetBoolean("TractionControl/Enable", mTractionControlEnabled);
  mTractionSlipThreshold = frc::SmartDashboard::GetNumber("TractionControl/SlipThreshold", mTractionSlipThreshold);
  mTractionKp = frc::SmartDashboard::GetNumber("TractionControl/Kp", mTractionKp);
  mYawStabilityKp = frc::SmartDashboard::GetNumber("TractionControl/YawKp", mYawStabilityKp);
  mDriftTorqueVectorScale = frc::SmartDashboard::GetNumber("Drift/TorqueVectorScale", mDriftTorqueVectorScale);

  double newDriveRatio = frc::SmartDashboard::GetNumber("Drive/DriveGearRatio", mDriveGearRatio);
  if (std::abs(newDriveRatio - mDriveGearRatio) > 1e-6) {
    mDriveGearRatio = newDriveRatio;
  }

  if (!frc::DriverStation::IsTest()) {
    return;
  }

  double newP = frc::SmartDashboard::GetNumber("Steer/kP", mP);
  double newI = frc::SmartDashboard::GetNumber("Steer/kI", mI);
  double newD = frc::SmartDashboard::GetNumber("Steer/kD", mD);
  double newS = frc::SmartDashboard::GetNumber("Steer/kS", mS);

  if (std::abs(newP - mP) > 1e-6 || std::abs(newI - mI) > 1e-6 || std::abs(newD - mD) > 1e-6 || std::abs(newS - mS) > 1e-6) {
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

  double newFLMagnetOffset = frc::SmartDashboard::GetNumber("Steer/FLMagnetOffset", mFLMagnetOffset);
  if (std::abs(newFLMagnetOffset - mFLMagnetOffset) > 1e-6) {
    mFLMagnetOffset = newFLMagnetOffset;
    ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
    magConfig.MagnetOffset = units::angle::turn_t{mFLMagnetOffset};
    magConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFLEncoder->GetConfigurator().Apply(magConfig);
    mFLEncoder->GetPosition().WaitForUpdate(250_ms);
    mFLSteer->SetPosition(mFLEncoder->GetPosition().GetValue());
  }

  double newFRMagnetOffset = frc::SmartDashboard::GetNumber("Steer/FRMagnetOffset", mFRMagnetOffset);
  if (std::abs(newFRMagnetOffset - mFRMagnetOffset) > 1e-6) {
    mFRMagnetOffset = newFRMagnetOffset;
    ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
    magConfig.MagnetOffset = units::angle::turn_t{mFRMagnetOffset};
    magConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFREncoder->GetConfigurator().Apply(magConfig);
    mFREncoder->GetPosition().WaitForUpdate(250_ms);
    mFRSteer->SetPosition(mFREncoder->GetPosition().GetValue());
  }

  bool newFLDriveInverted = frc::SmartDashboard::GetBoolean("Drive/FLDriveInverted", mFLDriveInverted);
  if (newFLDriveInverted != mFLDriveInverted) {
    mFLDriveInverted = newFLDriveInverted;
    ctre::phoenix6::configs::MotorOutputConfigs outputConfig{};
    outputConfig.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    outputConfig.Inverted = mFLDriveInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    mFLDrive->GetConfigurator().Apply(outputConfig);
  }

  bool newFRDriveInverted = frc::SmartDashboard::GetBoolean("Drive/FRDriveInverted", mFRDriveInverted);
  if (newFRDriveInverted != mFRDriveInverted) {
    mFRDriveInverted = newFRDriveInverted;
    ctre::phoenix6::configs::MotorOutputConfigs outputConfig{};
    outputConfig.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    outputConfig.Inverted = mFRDriveInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    mFRDrive->GetConfigurator().Apply(outputConfig);
  }

  bool newFLSteerInverted = frc::SmartDashboard::GetBoolean("Steer/FLSteerInverted", mFLSteerInverted);
  if (newFLSteerInverted != mFLSteerInverted) {
    mFLSteerInverted = newFLSteerInverted;
    ctre::phoenix6::configs::MotorOutputConfigs outputConfig{};
    outputConfig.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    outputConfig.Inverted = mFLSteerInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    mFLSteer->GetConfigurator().Apply(outputConfig);
  }

  bool newFRSteerInverted = frc::SmartDashboard::GetBoolean("Steer/FRSteerInverted", mFRSteerInverted);
  if (newFRSteerInverted != mFRSteerInverted) {
    mFRSteerInverted = newFRSteerInverted;
    ctre::phoenix6::configs::MotorOutputConfigs outputConfig{};
    outputConfig.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    outputConfig.Inverted = mFRSteerInverted
        ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
        : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    mFRSteer->GetConfigurator().Apply(outputConfig);
  }

  double newGearRatio = frc::SmartDashboard::GetNumber("Steer/GearRatio", mSteerGearRatio);
  if (std::abs(newGearRatio - mSteerGearRatio) > 1e-6) {
    mSteerGearRatio = newGearRatio;
    ctre::phoenix6::configs::FeedbackConfigs feedbackConfig{};
    feedbackConfig.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
    feedbackConfig.SensorToMechanismRatio = mSteerGearRatio;
    mFLSteer->GetConfigurator().Apply(feedbackConfig);
    mFRSteer->GetConfigurator().Apply(feedbackConfig);
    syncSteerToCANcoder();
  }

  if (frc::SmartDashboard::GetBoolean("Steer/SyncCANcoder", false)) {
    syncSteerToCANcoder();
    frc::SmartDashboard::PutBoolean("Steer/SyncCANcoder", false);
  }
}

void SubCarDrive::updateTelemetry() {
  double targetDeg = mTargetSteerAngle.value();
  double flDeg = units::angle::degree_t{mFLSteer->GetPosition().GetValue()}.value();
  double frDeg = units::angle::degree_t{mFRSteer->GetPosition().GetValue()}.value();

  frc::SmartDashboard::PutNumber("Steer/TargetAngleDeg", targetDeg);
  frc::SmartDashboard::PutNumber("Steer/FLTargetDeg", mTargetSteerAngleFL.value());
  frc::SmartDashboard::PutNumber("Steer/FRTargetDeg", mTargetSteerAngleFR.value());
  frc::SmartDashboard::PutNumber("Drive/FLTargetSpeed", mTargetSpeedFL);
  frc::SmartDashboard::PutNumber("Drive/FRTargetSpeed", mTargetSpeedFR);

  frc::SmartDashboard::PutNumber("Steer/FLAngleDeg", flDeg);
  frc::SmartDashboard::PutNumber("Steer/FRAngleDeg", frDeg);
  frc::SmartDashboard::PutNumber("Steer/FLErrDeg", mTargetSteerAngleFL.value() - flDeg);
  frc::SmartDashboard::PutNumber("Steer/FRErrDeg", mTargetSteerAngleFR.value() - frDeg);

  frc::SmartDashboard::PutNumber("Steer/FLEncoderDeg", units::angle::degree_t{mFLEncoder->GetPosition().GetValue()}.value());
  frc::SmartDashboard::PutNumber("Steer/FREncoderDeg", units::angle::degree_t{mFREncoder->GetPosition().GetValue()}.value());

  frc::SmartDashboard::PutBoolean("Steer/FLCANcoderConnected", mFLEncoder->IsConnected());
  frc::SmartDashboard::PutBoolean("Steer/FRCANcoderConnected", mFREncoder->IsConnected());

  frc::SmartDashboard::PutNumber("Steer/FLCurrentAmps", mFLSteer->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Steer/FRCurrentAmps", mFRSteer->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Drive/FLCurrentAmps", mFLDrive->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Drive/FRCurrentAmps", mFRDrive->GetStatorCurrent().GetValue().value());

  frc::SmartDashboard::PutBoolean("TractionControl/IsSlipping", mIsSlipping);
  frc::SmartDashboard::PutBoolean("Drift/Active", mDriftActive);

  units::angle::degree_t yaw = mPigeon->GetYaw().GetValue();
  frc::SmartDashboard::PutNumber("IMU/YawDeg", yaw.value());
  units::acceleration::meters_per_second_squared_t ax = mPigeon->GetAccelerationX().GetValue();
  frc::SmartDashboard::PutNumber("IMU/AccelX", ax.value());
  units::angular_velocity::degrees_per_second_t gz = mPigeon->GetAngularVelocityZDevice().GetValue();
  frc::SmartDashboard::PutNumber("IMU/YawRateDps", gz.value());
}
