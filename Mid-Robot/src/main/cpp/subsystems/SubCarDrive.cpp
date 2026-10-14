#include "subsystems/SubCarDrive.h"

#include <algorithm>
#include <cmath>
#include <ctre/phoenix6/configs/MagnetSensorConfigs.hpp>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>
#include "Constants.h"

SubCarDrive::SubCarDrive() {
  mFLDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFLDrive};
  mFLSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFLSteer};
  mFLEncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFLEncoder};

  mFRDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFRDrive};
  mFRSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFRSteer};
  mFREncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFREncoder};

  mPigeon = new ctre::phoenix6::hardware::Pigeon2{CANid::kPigeon2};

  applyMagnetConfig(mFLEncoder, mFLMagnetOffset);
  applyMagnetConfig(mFREncoder, mFRMagnetOffset);
  applyDriveConfig(mFLDrive, mFLDriveInverted);
  applyDriveConfig(mFRDrive, mFRDriveInverted);
  applySteerConfig(mFLSteer, mFLSteerInverted);
  applySteerConfig(mFRSteer, mFRSteerInverted);

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
  units::meter_t halfTrack = mTrackWidth / 2.0;

  double speedFL = speed;
  double speedFR = speed;
  units::angle::degree_t angleFL{0.0_deg};
  units::angle::degree_t angleFR{0.0_deg};

  if (units::math::abs(delta) > 1e-4_rad) {
    units::meter_t turnRadius = mWheelBase / units::math::tan(delta);

    angleFL = units::math::atan(mWheelBase / (turnRadius - halfTrack));
    angleFR = units::math::atan(mWheelBase / (turnRadius + halfTrack));

    angleFL = std::clamp(angleFL, -mMaxSteerAngle, mMaxSteerAngle);
    angleFR = std::clamp(angleFR, -mMaxSteerAngle, mMaxSteerAngle);

    units::meter_t absRadius = units::math::abs(turnRadius);
    units::meter_t centerDistance = units::math::hypot(mWheelBase, absRadius);
    double ratioOuter = (units::math::hypot(mWheelBase, absRadius + halfTrack) / centerDistance).value();
    double ratioInner = (units::math::hypot(mWheelBase, std::max(0_m, absRadius - halfTrack)) / centerDistance).value();

    double normSpeed = std::abs(speed);
    double maxRatio = normSpeed * ratioOuter;
    if (maxRatio > 1.0 && normSpeed > 1e-4) {
      ratioOuter /= maxRatio;
      ratioInner /= maxRatio;
    }

    double speedOuter = speed * ratioOuter;
    double speedInner = speed * ratioInner;

    if (turnRadius < 0_m) {
      speedFR = speedOuter;
      speedFL = speedInner;
    } else {
      speedFL = speedOuter;
      speedFR = speedInner;
    }
  }

  mDriftActive = iDrift;

  if (std::abs(speed) < 1e-4) {
    speedFL = 0.0;
    speedFR = 0.0;
  } else if (iDrift) {
    if (steerAngle > 1.0_deg) {
      speedFL = std::clamp(speedFL * mDriftTorqueVectorScale, -1.0, 1.0);
      speedFR *= (1.0 - std::abs(iSteer) * CarDriveConstants::kDriftInnerScale);
    } else if (steerAngle < -1.0_deg) {
      speedFR = std::clamp(speedFR * mDriftTorqueVectorScale, -1.0, 1.0);
      speedFL *= (1.0 - std::abs(iSteer) * CarDriveConstants::kDriftInnerScale);
    }
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

void SubCarDrive::applyMagnetConfig(ctre::phoenix6::hardware::CANcoder* iEncoder, units::angle::turn_t iOffset) {
  ctre::phoenix6::configs::MagnetSensorConfigs magnetConfig{};
  magnetConfig.MagnetOffset = iOffset;
  magnetConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  iEncoder->GetConfigurator().Apply(magnetConfig);
}

void SubCarDrive::applyDriveConfig(ctre::phoenix6::hardware::TalonFX* iMotor, bool iInverted) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.MotorOutput.Inverted = iInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  iMotor->GetConfigurator().Apply(config);
}

void SubCarDrive::applySteerConfig(ctre::phoenix6::hardware::TalonFX* iMotor, bool iInverted) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.MotorOutput.Inverted = iInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  config.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
  config.Feedback.SensorToMechanismRatio = mSteerGearRatio;
  config.ClosedLoopGeneral.ContinuousWrap = true;
  config.Slot0.kP = mSteerP;
  config.Slot0.kI = mSteerI;
  config.Slot0.kD = mSteerD;
  config.Slot0.kS = mSteerS;
  iMotor->GetConfigurator().Apply(config);
}

void SubCarDrive::zeroSteer(ctre::phoenix6::hardware::CANcoder* iEncoder, ctre::phoenix6::hardware::TalonFX* iSteer,
                            units::angle::turn_t& ioMagnetOffset, std::string_view iKey) {
  units::angle::turn_t absoluteTurns = iEncoder->GetAbsolutePosition().GetValue();
  ioMagnetOffset = units::angle::turn_t{std::remainder((ioMagnetOffset - absoluteTurns).value(), 1.0)};

  applyMagnetConfig(iEncoder, ioMagnetOffset);

  iEncoder->GetPosition().WaitForUpdate(250_ms);
  iSteer->SetPosition(iEncoder->GetPosition().GetValue());

  frc::SmartDashboard::PutNumber(iKey, ioMagnetOffset.value());
}

void SubCarDrive::zeroFL() {
  zeroSteer(mFLEncoder, mFLSteer, mFLMagnetOffset, "Steer/FLMagnetOffset");
}

void SubCarDrive::zeroFR() {
  zeroSteer(mFREncoder, mFRSteer, mFRMagnetOffset, "Steer/FRMagnetOffset");
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
    stop();
  });
}

void SubCarDrive::initDashboard() {
  frc::SmartDashboard::SetDefaultBoolean("Steer/SyncCANcoder", false);
  frc::SmartDashboard::SetDefaultNumber("Steer/kP", mSteerP);
  frc::SmartDashboard::SetDefaultNumber("Steer/kI", mSteerI);
  frc::SmartDashboard::SetDefaultNumber("Steer/kD", mSteerD);
  frc::SmartDashboard::SetDefaultNumber("Steer/kS", mSteerS);

  frc::SmartDashboard::SetDefaultNumber("Steer/FLMagnetOffset", mFLMagnetOffset.value());
  frc::SmartDashboard::SetDefaultNumber("Steer/FRMagnetOffset", mFRMagnetOffset.value());

  frc::SmartDashboard::SetDefaultBoolean("Drive/FLDriveInverted", mFLDriveInverted);
  frc::SmartDashboard::SetDefaultBoolean("Drive/FRDriveInverted", mFRDriveInverted);
  frc::SmartDashboard::SetDefaultBoolean("Steer/FLSteerInverted", mFLSteerInverted);
  frc::SmartDashboard::SetDefaultBoolean("Steer/FRSteerInverted", mFRSteerInverted);

  frc::SmartDashboard::SetDefaultNumber("Steer/GearRatio", mSteerGearRatio);
  frc::SmartDashboard::SetDefaultNumber("Steer/MaxAngleDeg", mMaxSteerAngle.value());
  frc::SmartDashboard::SetDefaultNumber("Drive/SpeedScale", mSpeedScale);

  frc::SmartDashboard::SetDefaultNumber("CarDrive/TrackWidthInches", units::inch_t{mTrackWidth}.value());
  frc::SmartDashboard::SetDefaultNumber("CarDrive/WheelBaseInches", units::inch_t{mWheelBase}.value());
  frc::SmartDashboard::SetDefaultNumber("Steer/TestTargetAngleDeg", 45.0);
  frc::SmartDashboard::SetDefaultBoolean("Steer/ZeroFL", false);
  frc::SmartDashboard::SetDefaultBoolean("Steer/ZeroFR", false);

  frc::SmartDashboard::SetDefaultNumber("Drift/TorqueVectorScale", mDriftTorqueVectorScale);
}

void SubCarDrive::updateConfigsFromDashboard() {
  double trackWidthInches = frc::SmartDashboard::GetNumber("CarDrive/TrackWidthInches", units::inch_t{mTrackWidth}.value());
  if (std::abs(trackWidthInches - units::inch_t{mTrackWidth}.value()) > 1e-6) {
    mTrackWidth = units::inch_t{trackWidthInches};
  }

  double wheelBaseInches = frc::SmartDashboard::GetNumber("CarDrive/WheelBaseInches", units::inch_t{mWheelBase}.value());
  if (std::abs(wheelBaseInches - units::inch_t{mWheelBase}.value()) > 1e-6) {
    mWheelBase = units::inch_t{wheelBaseInches};
  }

  double speedScale = frc::SmartDashboard::GetNumber("Drive/SpeedScale", mSpeedScale);
  if (std::abs(speedScale - mSpeedScale) > 1e-6) {
    mSpeedScale = speedScale;
  }

  double maxSteerAngleDeg = frc::SmartDashboard::GetNumber("Steer/MaxAngleDeg", mMaxSteerAngle.value());
  if (std::abs(maxSteerAngleDeg - mMaxSteerAngle.value()) > 1e-6) {
    mMaxSteerAngle = units::angle::degree_t{maxSteerAngleDeg};
  }

  double driftTorqueVectorScale = frc::SmartDashboard::GetNumber("Drift/TorqueVectorScale", mDriftTorqueVectorScale);
  if (std::abs(driftTorqueVectorScale - mDriftTorqueVectorScale) > 1e-6) {
    mDriftTorqueVectorScale = driftTorqueVectorScale;
  }

  if (!frc::DriverStation::IsTest()) {
    return;
  }

  double steerP = frc::SmartDashboard::GetNumber("Steer/kP", mSteerP);
  double steerI = frc::SmartDashboard::GetNumber("Steer/kI", mSteerI);
  double steerD = frc::SmartDashboard::GetNumber("Steer/kD", mSteerD);
  double steerS = frc::SmartDashboard::GetNumber("Steer/kS", mSteerS);
  if (std::abs(steerP - mSteerP) > 1e-6 || std::abs(steerI - mSteerI) > 1e-6 ||
      std::abs(steerD - mSteerD) > 1e-6 || std::abs(steerS - mSteerS) > 1e-6) {
    mSteerP = steerP;
    mSteerI = steerI;
    mSteerD = steerD;
    mSteerS = steerS;
    applySteerConfig(mFLSteer, mFLSteerInverted);
    applySteerConfig(mFRSteer, mFRSteerInverted);
  }

  if (frc::SmartDashboard::GetBoolean("Steer/ZeroFL", false)) {
    zeroFL();
    frc::SmartDashboard::PutBoolean("Steer/ZeroFL", false);
  }

  if (frc::SmartDashboard::GetBoolean("Steer/ZeroFR", false)) {
    zeroFR();
    frc::SmartDashboard::PutBoolean("Steer/ZeroFR", false);
  }

  double flMagnetOffset = frc::SmartDashboard::GetNumber("Steer/FLMagnetOffset", mFLMagnetOffset.value());
  if (std::abs(flMagnetOffset - mFLMagnetOffset.value()) > 1e-6) {
    mFLMagnetOffset = units::angle::turn_t{flMagnetOffset};
    applyMagnetConfig(mFLEncoder, mFLMagnetOffset);
    syncSteerToCANcoder();
  }

  double frMagnetOffset = frc::SmartDashboard::GetNumber("Steer/FRMagnetOffset", mFRMagnetOffset.value());
  if (std::abs(frMagnetOffset - mFRMagnetOffset.value()) > 1e-6) {
    mFRMagnetOffset = units::angle::turn_t{frMagnetOffset};
    applyMagnetConfig(mFREncoder, mFRMagnetOffset);
    syncSteerToCANcoder();
  }

  bool flDriveInverted = frc::SmartDashboard::GetBoolean("Drive/FLDriveInverted", mFLDriveInverted);
  if (flDriveInverted != mFLDriveInverted) {
    mFLDriveInverted = flDriveInverted;
    applyDriveConfig(mFLDrive, mFLDriveInverted);
  }

  bool frDriveInverted = frc::SmartDashboard::GetBoolean("Drive/FRDriveInverted", mFRDriveInverted);
  if (frDriveInverted != mFRDriveInverted) {
    mFRDriveInverted = frDriveInverted;
    applyDriveConfig(mFRDrive, mFRDriveInverted);
  }

  bool flSteerInverted = frc::SmartDashboard::GetBoolean("Steer/FLSteerInverted", mFLSteerInverted);
  if (flSteerInverted != mFLSteerInverted) {
    mFLSteerInverted = flSteerInverted;
    applySteerConfig(mFLSteer, mFLSteerInverted);
  }

  bool frSteerInverted = frc::SmartDashboard::GetBoolean("Steer/FRSteerInverted", mFRSteerInverted);
  if (frSteerInverted != mFRSteerInverted) {
    mFRSteerInverted = frSteerInverted;
    applySteerConfig(mFRSteer, mFRSteerInverted);
  }

  double steerGearRatio = frc::SmartDashboard::GetNumber("Steer/GearRatio", mSteerGearRatio);
  if (std::abs(steerGearRatio - mSteerGearRatio) > 1e-6) {
    mSteerGearRatio = steerGearRatio;
    applySteerConfig(mFLSteer, mFLSteerInverted);
    applySteerConfig(mFRSteer, mFRSteerInverted);
    syncSteerToCANcoder();
  }

  if (frc::SmartDashboard::GetBoolean("Steer/SyncCANcoder", false)) {
    syncSteerToCANcoder();
    frc::SmartDashboard::PutBoolean("Steer/SyncCANcoder", false);
  }
}

void SubCarDrive::updateTelemetry() {
  mTelemetryCounter++;
  if (mTelemetryCounter < 5) {
    return;
  }
  mTelemetryCounter = 0;

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
  frc::SmartDashboard::PutNumber("Steer/FLMotorVoltage", mFLSteer->GetMotorVoltage().GetValue().value());
  frc::SmartDashboard::PutNumber("Steer/FRMotorVoltage", mFRSteer->GetMotorVoltage().GetValue().value());
  frc::SmartDashboard::PutNumber("Drive/FLCurrentAmps", mFLDrive->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Drive/FRCurrentAmps", mFRDrive->GetStatorCurrent().GetValue().value());

  frc::SmartDashboard::PutBoolean("Drift/Active", mDriftActive);

  units::angle::degree_t yaw = mPigeon->GetYaw().GetValue();
  frc::SmartDashboard::PutNumber("IMU/YawDeg", yaw.value());
  units::acceleration::meters_per_second_squared_t ax = mPigeon->GetAccelerationX().GetValue();
  frc::SmartDashboard::PutNumber("IMU/AccelX", ax.value());
  units::angular_velocity::degrees_per_second_t gz = mPigeon->GetAngularVelocityZDevice().GetValue();
  frc::SmartDashboard::PutNumber("IMU/YawRateDps", gz.value());
}
