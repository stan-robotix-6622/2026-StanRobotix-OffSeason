#include "subsystems/SubCarDrive.h"

#include <algorithm>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include "Constants.h"
#include "ctre/phoenix6/configs/Slot0Configs.hpp"
#include "ctre/phoenix6/configs/ClosedLoopGeneralConfigs.hpp"
#include "ctre/phoenix6/configs/MagnetSensorConfigs.hpp"
#include "ctre/phoenix6/configs/MotorOutputConfigs.hpp"
#include "ctre/phoenix6/configs/FeedbackConfigs.hpp"

SubCarDrive::SubCarDrive() {
  mFLDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFLDrive};
  mFLSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFLSteer};
  mFLEncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFLEncoder};

  mFRDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFRDrive};
  mFRSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFRSteer};
  mFREncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFREncoder};

  mP = CarDriveConstants::kSteerP;
  mI = CarDriveConstants::kSteerI;
  mD = CarDriveConstants::kSteerD;
  mS = CarDriveConstants::kSteerS;
  mSteerTolerance = CarDriveConstants::kSteerTolerance;

  mFLMagnetOffset = CarDriveConstants::kFLMagnetOffset.value();
  mFRMagnetOffset = CarDriveConstants::kFRMagnetOffset.value();

  mFLDriveInverted = CarDriveConstants::kFLDriveInverted;
  mFRDriveInverted = CarDriveConstants::kFRDriveInverted;
  mFLSteerInverted = CarDriveConstants::kFLSteerInverted;
  mFRSteerInverted = CarDriveConstants::kFRSteerInverted;

  mSteerGearRatio = CarDriveConstants::kSteerGearRatio;
  mMaxSteerAngle = CarDriveConstants::kMaxSteerAngle;
  mSpeedScale = CarDriveConstants::kSpeedScale;

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
  steerConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  steerConfig.Feedback.FeedbackRemoteSensorID = CANid::kFLEncoder;
  steerConfig.Feedback.RotorToSensorRatio = mSteerGearRatio;
  steerConfig.Feedback.SensorToMechanismRatio = 1.0;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
  steerConfig.ClosedLoopGeneral.GainSchedErrorThreshold = mSteerTolerance;
  steerConfig.Slot0.GainSchedBehavior = ctre::phoenix6::signals::GainSchedBehaviorValue::ZeroOutput;
  steerConfig.Slot0.kP = mP;
  steerConfig.Slot0.kI = mI;
  steerConfig.Slot0.kD = mD;
  steerConfig.Slot0.kS = mS;
  mFLSteer->GetConfigurator().Apply(steerConfig);

  steerConfig.MotorOutput.Inverted = mFRSteerInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.Feedback.FeedbackRemoteSensorID = CANid::kFREncoder;
  mFRSteer->GetConfigurator().Apply(steerConfig);

  mSteerPositionControl.OverrideBrakeDurNeutral = true;

  initDashboard();
}

SubCarDrive::~SubCarDrive() {
  delete mFLDrive;
  delete mFLSteer;
  delete mFLEncoder;

  delete mFRDrive;
  delete mFRSteer;
  delete mFREncoder;
}

void SubCarDrive::Periodic() {
  updateConfigsFromDashboard();
  updateTelemetry();
}

void SubCarDrive::drive(double iThrottle, double iBrake, double iSteer) {
  if (frc::SmartDashboard::GetBoolean("Steer/TestEnable", false)) {
    return;
  }

  double throttle = std::clamp(iThrottle, 0.0, 1.0);
  double brake = std::clamp(iBrake, 0.0, 1.0);
  double speed = (throttle * mSpeedScale) * (1.0 - brake);
  if (speed < 0.0) {
    speed = 0.0;
  }

  units::angle::degree_t steerAngle = -std::clamp(iSteer, -1.0, 1.0) * mMaxSteerAngle;
  setSteerAngle(steerAngle);

  mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(speed));
  mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(speed));
}

void SubCarDrive::stop() {
  mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
  mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
}

void SubCarDrive::stopDrive() {
  stop();
}

void SubCarDrive::setSteerAngle(units::angle::degree_t iAngle) {
  units::angle::degree_t clampedAngle = std::clamp(iAngle, -mMaxSteerAngle, mMaxSteerAngle);
  mTargetSteerAngle = clampedAngle;

  double targetDeg = clampedAngle.value();
  double flDeg = units::angle::degree_t{mFLSteer->GetPosition().GetValue()}.value();
  double frDeg = units::angle::degree_t{mFRSteer->GetPosition().GetValue()}.value();
  double flErr = std::abs(targetDeg - flDeg);
  double frErr = std::abs(targetDeg - frDeg);
  double tol = mSteerTolerance.value();

  if (mFLAtTarget) {
    if (flErr > tol + 0.5) {
      mFLAtTarget = false;
    }
  } else {
    if (flErr <= tol) {
      mFLAtTarget = true;
    }
  }

  if (mFRAtTarget) {
    if (frErr > tol + 0.5) {
      mFRAtTarget = false;
    }
  } else {
    if (frErr <= tol) {
      mFRAtTarget = true;
    }
  }

  if (mFLAtTarget) {
    mFLSteer->SetControl(mSteerNeutralControl);
  } else {
    mFLSteer->SetControl(mSteerPositionControl.WithPosition(clampedAngle));
  }

  if (mFRAtTarget) {
    mFRSteer->SetControl(mSteerNeutralControl);
  } else {
    mFRSteer->SetControl(mSteerPositionControl.WithPosition(clampedAngle));
  }
}

frc2::CommandPtr SubCarDrive::getTestSteerCommand() {
  return Run([this] {
    double targetAngleDeg = frc::SmartDashboard::GetNumber("Steer/TestTargetAngleDeg", 45.0);
    setSteerAngle(units::angle::degree_t{targetAngleDeg});
    stopDrive();
  });
}

void SubCarDrive::initDashboard() {
  frc::SmartDashboard::SetDefaultNumber("Steer/kP", mP);
  frc::SmartDashboard::SetDefaultNumber("Steer/kI", mI);
  frc::SmartDashboard::SetDefaultNumber("Steer/kD", mD);
  frc::SmartDashboard::SetDefaultNumber("Steer/kS", mS);
  frc::SmartDashboard::SetDefaultNumber("Steer/ToleranceDeg", mSteerTolerance.value());

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

void SubCarDrive::updateConfigsFromDashboard() {
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
    slot0Config.GainSchedBehavior = ctre::phoenix6::signals::GainSchedBehaviorValue::ZeroOutput;
    slot0Config.kP = mP;
    slot0Config.kI = mI;
    slot0Config.kD = mD;
    slot0Config.kS = mS;
    mFLSteer->GetConfigurator().Apply(slot0Config);
    mFRSteer->GetConfigurator().Apply(slot0Config);
  }

  if (frc::SmartDashboard::GetBoolean("Steer/ZeroFL", false)) {
    double currentPos = mFLEncoder->GetAbsolutePosition().GetValue().value();
    mFLMagnetOffset -= currentPos;
    while (mFLMagnetOffset > 1.0) mFLMagnetOffset -= 1.0;
    while (mFLMagnetOffset < -1.0) mFLMagnetOffset += 1.0;

    ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
    magConfig.MagnetOffset = units::angle::turn_t{mFLMagnetOffset};
    magConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFLEncoder->GetConfigurator().Apply(magConfig);

    frc::SmartDashboard::PutNumber("Steer/FLMagnetOffset", mFLMagnetOffset);
    frc::SmartDashboard::PutBoolean("Steer/ZeroFL", false);
  }

  if (frc::SmartDashboard::GetBoolean("Steer/ZeroFR", false)) {
    double currentPos = mFREncoder->GetAbsolutePosition().GetValue().value();
    mFRMagnetOffset -= currentPos;
    while (mFRMagnetOffset > 1.0) mFRMagnetOffset -= 1.0;
    while (mFRMagnetOffset < -1.0) mFRMagnetOffset += 1.0;

    ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
    magConfig.MagnetOffset = units::angle::turn_t{mFRMagnetOffset};
    magConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFREncoder->GetConfigurator().Apply(magConfig);

    frc::SmartDashboard::PutNumber("Steer/FRMagnetOffset", mFRMagnetOffset);
    frc::SmartDashboard::PutBoolean("Steer/ZeroFR", false);
  }

  double newFLMagnetOffset = frc::SmartDashboard::GetNumber("Steer/FLMagnetOffset", mFLMagnetOffset);
  if (std::abs(newFLMagnetOffset - mFLMagnetOffset) > 1e-6) {
    mFLMagnetOffset = newFLMagnetOffset;
    ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
    magConfig.MagnetOffset = units::angle::turn_t{mFLMagnetOffset};
    magConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFLEncoder->GetConfigurator().Apply(magConfig);
  }

  double newFRMagnetOffset = frc::SmartDashboard::GetNumber("Steer/FRMagnetOffset", mFRMagnetOffset);
  if (std::abs(newFRMagnetOffset - mFRMagnetOffset) > 1e-6) {
    mFRMagnetOffset = newFRMagnetOffset;
    ctre::phoenix6::configs::MagnetSensorConfigs magConfig{};
    magConfig.MagnetOffset = units::angle::turn_t{mFRMagnetOffset};
    magConfig.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    mFREncoder->GetConfigurator().Apply(magConfig);
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
    feedbackConfig.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    feedbackConfig.FeedbackRemoteSensorID = CANid::kFLEncoder;
    feedbackConfig.RotorToSensorRatio = mSteerGearRatio;
    feedbackConfig.SensorToMechanismRatio = 1.0;
    mFLSteer->GetConfigurator().Apply(feedbackConfig);

    feedbackConfig.FeedbackRemoteSensorID = CANid::kFREncoder;
    mFRSteer->GetConfigurator().Apply(feedbackConfig);
  }

  double newMaxSteerAngleDeg = frc::SmartDashboard::GetNumber("Steer/MaxAngleDeg", mMaxSteerAngle.value());
  if (std::abs(newMaxSteerAngleDeg - mMaxSteerAngle.value()) > 1e-6) {
    mMaxSteerAngle = units::angle::degree_t{newMaxSteerAngleDeg};
  }

  double newSpeedScale = frc::SmartDashboard::GetNumber("Drive/SpeedScale", mSpeedScale);
  if (std::abs(newSpeedScale - mSpeedScale) > 1e-6) {
    mSpeedScale = newSpeedScale;
  }

  double newToleranceDeg = frc::SmartDashboard::GetNumber("Steer/ToleranceDeg", mSteerTolerance.value());
  if (std::abs(newToleranceDeg - mSteerTolerance.value()) > 1e-6) {
    mSteerTolerance = units::angle::degree_t{newToleranceDeg};
    ctre::phoenix6::configs::ClosedLoopGeneralConfigs generalConfig{};
    generalConfig.ContinuousWrap = true;
    generalConfig.GainSchedErrorThreshold = mSteerTolerance;
    mFLSteer->GetConfigurator().Apply(generalConfig);
    mFRSteer->GetConfigurator().Apply(generalConfig);
  }
}

void SubCarDrive::updateTelemetry() {
  if (frc::SmartDashboard::GetBoolean("Steer/TestEnable", false)) {
    double targetAngleDeg = frc::SmartDashboard::GetNumber("Steer/TestTargetAngleDeg", 45.0);
    setSteerAngle(units::angle::degree_t{targetAngleDeg});
    stopDrive();
  }

  double targetDeg = mTargetSteerAngle.value();
  double flDeg = units::angle::degree_t{mFLSteer->GetPosition().GetValue()}.value();
  double frDeg = units::angle::degree_t{mFRSteer->GetPosition().GetValue()}.value();

  frc::SmartDashboard::PutNumber("Steer/TargetAngleDeg", targetDeg);
  frc::SmartDashboard::PutNumber("Steer/FLAngleDeg", flDeg);
  frc::SmartDashboard::PutNumber("Steer/FRAngleDeg", frDeg);
  frc::SmartDashboard::PutNumber("Steer/FLErrDeg", targetDeg - flDeg);
  frc::SmartDashboard::PutNumber("Steer/FRErrDeg", targetDeg - frDeg);
  frc::SmartDashboard::PutBoolean("Steer/FLAtTarget", mFLAtTarget);
  frc::SmartDashboard::PutBoolean("Steer/FRAtTarget", mFRAtTarget);

  frc::SmartDashboard::PutNumber("Steer/FLEncoderDeg", units::angle::degree_t{mFLEncoder->GetPosition().GetValue()}.value());
  frc::SmartDashboard::PutNumber("Steer/FREncoderDeg", units::angle::degree_t{mFREncoder->GetPosition().GetValue()}.value());

  frc::SmartDashboard::PutBoolean("Steer/FLCANcoderConnected", mFLEncoder->IsConnected());
  frc::SmartDashboard::PutBoolean("Steer/FRCANcoderConnected", mFREncoder->IsConnected());

  frc::SmartDashboard::PutNumber("Steer/FLCurrentAmps", mFLSteer->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Steer/FRCurrentAmps", mFRSteer->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Drive/FLCurrentAmps", mFLDrive->GetStatorCurrent().GetValue().value());
  frc::SmartDashboard::PutNumber("Drive/FRCurrentAmps", mFRDrive->GetStatorCurrent().GetValue().value());
}
