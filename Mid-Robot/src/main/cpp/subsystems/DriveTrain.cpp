#include "subsystems/DriveTrain.h"

#include <algorithm>
#include <cmath>
#include <ctre/phoenix6/configs/MagnetSensorConfigs.hpp>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <units/math.h>

DriveTrain::DriveTrain() {
  mBLDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kBLDrive};
  mBLSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kBLSteer};
  mBLEncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kBLEncoder};

  mBRDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kBRDrive};
  mBRSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kBRSteer};
  mBREncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kBREncoder};

  mPigeon = new ctre::phoenix6::hardware::Pigeon2{CANid::kPigeon2};

  ApplyMagnetConfig(mBLEncoder, mBLMagnetOffset);
  ApplyMagnetConfig(mBREncoder, mBRMagnetOffset);
  ApplyDriveConfig(mBLDrive, mBLDriveInverted);
  ApplyDriveConfig(mBRDrive, mBRDriveInverted);
  ApplySteerConfig(mBLSteer, mBLSteerInverted);
  ApplySteerConfig(mBRSteer, mBRSteerInverted);

  SyncSteerToCANcoder();
  InitDashboard();
}

DriveTrain::~DriveTrain() {
  delete mBLDrive;
  delete mBLSteer;
  delete mBLEncoder;

  delete mBRDrive;
  delete mBRSteer;
  delete mBREncoder;

  delete mPigeon;
}

void DriveTrain::Periodic() {
  UpdateConfigsFromDashboard();
  UpdateTelemetry();
}

void DriveTrain::Drive(double iThrottle, double iBrake, double iSteer, bool iReverse, bool iDrift) {
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

  double speedBL = speed;
  double speedBR = speed;
  units::angle::degree_t angleBL{0.0_deg};
  units::angle::degree_t angleBR{0.0_deg};

  if (units::math::abs(delta) > 1e-4_rad) {
    units::meter_t turnRadius = mWheelBase / units::math::tan(delta);

    angleBL = units::math::atan(mWheelBase / (turnRadius - halfTrack));
    angleBR = units::math::atan(mWheelBase / (turnRadius + halfTrack));

    angleBL = std::clamp(angleBL, -mMaxSteerAngle, mMaxSteerAngle);
    angleBR = std::clamp(angleBR, -mMaxSteerAngle, mMaxSteerAngle);

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
      speedBR = speedOuter;
      speedBL = speedInner;
    } else {
      speedBL = speedOuter;
      speedBR = speedInner;
    }
  }

  mDriftActive = iDrift;

  if (std::abs(speed) < 1e-4) {
    speedBL = 0.0;
    speedBR = 0.0;
  } else if (iDrift) {
    // Mode drift dynamique pendant la conduite
    if (steerAngle > 1.0_deg) {
      speedBL = std::clamp(speedBL * mDriftTorqueVectorScale, -1.0, 1.0);
      speedBR *= (1.0 - std::abs(iSteer) * mDriftInnerScale);
    } else if (steerAngle < -1.0_deg) {
      speedBR = std::clamp(speedBR * mDriftTorqueVectorScale, -1.0, 1.0);
      speedBL *= (1.0 - std::abs(iSteer) * mDriftInnerScale);
    }
  }

  mTargetSteerAngleBL = angleBL;
  mTargetSteerAngleBR = angleBR;
  mTargetSpeedBL = speedBL;
  mTargetSpeedBR = speedBR;

  mBLDrive->SetControl(mDriveDutyCycleControl.WithOutput(speedBL));
  mBRDrive->SetControl(mDriveDutyCycleControl.WithOutput(speedBR));

  mBLSteer->SetControl(mSteerPositionControl.WithPosition(angleBL));
  mBRSteer->SetControl(mSteerPositionControl.WithPosition(angleBR));
}

void DriveTrain::DriveArcade(double iSpeed, double iRotation, bool iDrift) {
  double throttle = std::abs(iSpeed);
  bool reverse = (iSpeed < 0.0);
  Drive(throttle, 0.0, iRotation, reverse, iDrift);
}

void DriveTrain::DriftLeft(double throttle) {
  mDriftActive = true;
  double t = std::clamp(throttle, 0.2, 1.0);

  // Braquage agressif vers la gauche
  units::angle::degree_t steerAngle = -DriveConstants::kDriftSteerAngle;
  mTargetSteerAngle = steerAngle;
  mTargetSteerAngleBL = steerAngle;
  mTargetSteerAngleBR = steerAngle;

  // Roue extérieure (BR) accélérée à fond, roue intérieure (BL) freinée
  double speedBR = std::clamp(t * mDriftTorqueVectorScale, -1.0, 1.0);
  double speedBL = t * (1.0 - mDriftInnerScale) * 0.25;

  mTargetSpeedBL = speedBL;
  mTargetSpeedBR = speedBR;

  mBLDrive->SetControl(mDriveDutyCycleControl.WithOutput(speedBL));
  mBRDrive->SetControl(mDriveDutyCycleControl.WithOutput(speedBR));

  mBLSteer->SetControl(mSteerPositionControl.WithPosition(steerAngle));
  mBRSteer->SetControl(mSteerPositionControl.WithPosition(steerAngle));
}

void DriveTrain::DriftRight(double throttle) {
  mDriftActive = true;
  double t = std::clamp(throttle, 0.2, 1.0);

  // Braquage agressif vers la droite
  units::angle::degree_t steerAngle = DriveConstants::kDriftSteerAngle;
  mTargetSteerAngle = steerAngle;
  mTargetSteerAngleBL = steerAngle;
  mTargetSteerAngleBR = steerAngle;

  // Roue extérieure (BL) accélérée à fond, roue intérieure (BR) freinée
  double speedBL = std::clamp(t * mDriftTorqueVectorScale, -1.0, 1.0);
  double speedBR = t * (1.0 - mDriftInnerScale) * 0.25;

  mTargetSpeedBL = speedBL;
  mTargetSpeedBR = speedBR;

  mBLDrive->SetControl(mDriveDutyCycleControl.WithOutput(speedBL));
  mBRDrive->SetControl(mDriveDutyCycleControl.WithOutput(speedBR));

  mBLSteer->SetControl(mSteerPositionControl.WithPosition(steerAngle));
  mBRSteer->SetControl(mSteerPositionControl.WithPosition(steerAngle));
}

void DriveTrain::Stop() {
  mDriftActive = false;
  mTargetSpeedBL = 0.0;
  mTargetSpeedBR = 0.0;
  mBLDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
  mBRDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
}

void DriveTrain::SetSteerAngle(units::angle::degree_t iAngle) {
  units::angle::degree_t clampedAngle = std::clamp(iAngle, -mMaxSteerAngle, mMaxSteerAngle);
  mTargetSteerAngle = clampedAngle;
  mTargetSteerAngleBL = clampedAngle;
  mTargetSteerAngleBR = clampedAngle;
  mBLSteer->SetControl(mSteerPositionControl.WithPosition(clampedAngle));
  mBRSteer->SetControl(mSteerPositionControl.WithPosition(clampedAngle));
}

void DriveTrain::SyncSteerToCANcoder() {
  mBLEncoder->GetPosition().WaitForUpdate(250_ms);
  mBREncoder->GetPosition().WaitForUpdate(250_ms);

  mBLSteer->SetPosition(mBLEncoder->GetPosition().GetValue());
  mBRSteer->SetPosition(mBREncoder->GetPosition().GetValue());
}

void DriveTrain::ApplyMagnetConfig(ctre::phoenix6::hardware::CANcoder* iEncoder, units::angle::turn_t iOffset) {
  ctre::phoenix6::configs::MagnetSensorConfigs magnetConfig{};
  magnetConfig.MagnetOffset = iOffset;
  magnetConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  iEncoder->GetConfigurator().Apply(magnetConfig);
}

void DriveTrain::ApplyDriveConfig(ctre::phoenix6::hardware::TalonFX* iMotor, bool iInverted) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit = DriveConstants::kSupplyCurrentLimit;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit = DriveConstants::kStatorCurrentLimit;
  config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  config.MotorOutput.Inverted = iInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  iMotor->GetConfigurator().Apply(config);
}

void DriveTrain::ApplySteerConfig(ctre::phoenix6::hardware::TalonFX* iMotor, bool iInverted) {
  ctre::phoenix6::configs::TalonFXConfiguration config{};
  config.CurrentLimits.SupplyCurrentLimitEnable = true;
  config.CurrentLimits.SupplyCurrentLimit = DriveConstants::kSupplyCurrentLimit;
  config.CurrentLimits.StatorCurrentLimitEnable = true;
  config.CurrentLimits.StatorCurrentLimit = DriveConstants::kStatorCurrentLimit;
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

void DriveTrain::ZeroSteer(ctre::phoenix6::hardware::CANcoder* iEncoder, ctre::phoenix6::hardware::TalonFX* iSteer,
                           units::angle::turn_t& ioMagnetOffset, std::string_view iKey) {
  units::angle::turn_t absoluteTurns = iEncoder->GetAbsolutePosition().GetValue();
  ioMagnetOffset = units::angle::turn_t{std::remainder((ioMagnetOffset - absoluteTurns).value(), 1.0)};

  ApplyMagnetConfig(iEncoder, ioMagnetOffset);
  iEncoder->GetPosition().WaitForUpdate(250_ms);
  iSteer->SetPosition(iEncoder->GetPosition().GetValue());

  frc::SmartDashboard::PutNumber(iKey, ioMagnetOffset.value());
}

void DriveTrain::ZeroBL() {
  ZeroSteer(mBLEncoder, mBLSteer, mBLMagnetOffset, "Steer/BLMagnetOffset");
}

void DriveTrain::ZeroBR() {
  ZeroSteer(mBREncoder, mBRSteer, mBRMagnetOffset, "Steer/BRMagnetOffset");
}

frc2::CommandPtr DriveTrain::GetZeroBLCommand() {
  return RunOnce([this] { ZeroBL(); });
}

frc2::CommandPtr DriveTrain::GetZeroBRCommand() {
  return RunOnce([this] { ZeroBR(); });
}

void DriveTrain::InitDashboard() {
  frc::SmartDashboard::SetDefaultBoolean("Steer/SyncCANcoder", false);
  frc::SmartDashboard::SetDefaultNumber("Steer/kP", mSteerP);
  frc::SmartDashboard::SetDefaultNumber("Steer/kI", mSteerI);
  frc::SmartDashboard::SetDefaultNumber("Steer/kD", mSteerD);
  frc::SmartDashboard::SetDefaultNumber("Steer/kS", mSteerS);

  frc::SmartDashboard::SetDefaultNumber("Steer/BLMagnetOffset", mBLMagnetOffset.value());
  frc::SmartDashboard::SetDefaultNumber("Steer/BRMagnetOffset", mBRMagnetOffset.value());

  frc::SmartDashboard::SetDefaultBoolean("Drive/BLDriveInverted", mBLDriveInverted);
  frc::SmartDashboard::SetDefaultBoolean("Drive/BRDriveInverted", mBRDriveInverted);
  frc::SmartDashboard::SetDefaultBoolean("Steer/BLSteerInverted", mBLSteerInverted);
  frc::SmartDashboard::SetDefaultBoolean("Steer/BRSteerInverted", mBRSteerInverted);

  frc::SmartDashboard::SetDefaultNumber("Steer/GearRatio", mSteerGearRatio);
  frc::SmartDashboard::SetDefaultNumber("Steer/MaxAngleDeg", mMaxSteerAngle.value());
  frc::SmartDashboard::SetDefaultNumber("Drive/SpeedScale", mSpeedScale);

  frc::SmartDashboard::SetDefaultNumber("CarDrive/TrackWidthInches", units::inch_t{mTrackWidth}.value());
  frc::SmartDashboard::SetDefaultNumber("CarDrive/WheelBaseInches", units::inch_t{mWheelBase}.value());
  frc::SmartDashboard::SetDefaultBoolean("Steer/ZeroBL", false);
  frc::SmartDashboard::SetDefaultBoolean("Steer/ZeroBR", false);

  frc::SmartDashboard::SetDefaultNumber("Drift/TorqueVectorScale", mDriftTorqueVectorScale);
  frc::SmartDashboard::SetDefaultNumber("Drift/InnerScale", mDriftInnerScale);
}

void DriveTrain::UpdateConfigsFromDashboard() {
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

  double driftInnerScale = frc::SmartDashboard::GetNumber("Drift/InnerScale", mDriftInnerScale);
  if (std::abs(driftInnerScale - mDriftInnerScale) > 1e-6) {
    mDriftInnerScale = driftInnerScale;
  }

  if (frc::DriverStation::IsTest()) {
    if (frc::SmartDashboard::GetBoolean("Steer/ZeroBL", false)) {
      ZeroBL();
      frc::SmartDashboard::PutBoolean("Steer/ZeroBL", false);
    }
    if (frc::SmartDashboard::GetBoolean("Steer/ZeroBR", false)) {
      ZeroBR();
      frc::SmartDashboard::PutBoolean("Steer/ZeroBR", false);
    }
  }
}

void DriveTrain::UpdateTelemetry() {
  mTelemetryCounter++;
  if (mTelemetryCounter < 5) {
    return;
  }
  mTelemetryCounter = 0;

  double blDeg = units::angle::degree_t{mBLSteer->GetPosition().GetValue()}.value();
  double brDeg = units::angle::degree_t{mBRSteer->GetPosition().GetValue()}.value();

  frc::SmartDashboard::PutNumber("Steer/TargetAngleDeg", mTargetSteerAngle.value());
  frc::SmartDashboard::PutNumber("Steer/BLTargetDeg", mTargetSteerAngleBL.value());
  frc::SmartDashboard::PutNumber("Steer/BRTargetDeg", mTargetSteerAngleBR.value());
  frc::SmartDashboard::PutNumber("Drive/BLTargetSpeed", mTargetSpeedBL);
  frc::SmartDashboard::PutNumber("Drive/BRTargetSpeed", mTargetSpeedBR);

  frc::SmartDashboard::PutNumber("Steer/BLAngleDeg", blDeg);
  frc::SmartDashboard::PutNumber("Steer/BRAngleDeg", brDeg);

  frc::SmartDashboard::PutNumber("Drive/BLCurrentAmps", mBLDrive->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Drive/BRCurrentAmps", mBRDrive->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Steer/BLCurrentAmps", mBLSteer->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Steer/BRCurrentAmps", mBRSteer->GetStatorCurrent().GetValue().value());

  frc::SmartDashboard::PutBoolean("Drift/Active", mDriftActive);

  // PDP monitoring
  frc::SmartDashboard::PutNumber("PDP/BatteryVoltage", mPDP.GetVoltage());
  frc::SmartDashboard::PutNumber("PDP/TotalCurrentAmps", mPDP.GetTotalCurrent());

  // Pigeon 2 IMU
  units::angle::degree_t yaw = mPigeon->GetYaw().GetValue();
  frc::SmartDashboard::PutNumber("IMU/YawDeg", yaw.value());
  units::angular_velocity::degrees_per_second_t gz = mPigeon->GetAngularVelocityZDevice().GetValue();
  frc::SmartDashboard::PutNumber("IMU/YawRateDps", gz.value());

  // Limelight
  auto limelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
  bool hasTarget = limelightTable->GetNumber("tv", 0.0) >= 1.0;
  frc::SmartDashboard::PutBoolean("Limelight/HasTarget", hasTarget);
  if (hasTarget) {
    frc::SmartDashboard::PutNumber("Limelight/TargetOffsetDeg", limelightTable->GetNumber("tx", 0.0));
  }
}
