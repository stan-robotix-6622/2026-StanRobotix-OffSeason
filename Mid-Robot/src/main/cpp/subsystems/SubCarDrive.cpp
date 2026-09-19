#include "subsystems/SubCarDrive.h"

#include <algorithm>
#include <units/angle.h>
#include "Constants.h"

SubCarDrive::SubCarDrive() {
  mFrontLeftDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFrontLeftDrive};
  mFrontLeftSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFrontLeftSteer};
  mFrontLeftCANcoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFrontLeftCANcoder};

  mFrontRightDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFrontRightDrive};
  mFrontRightSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFrontRightSteer};
  mFrontRightCANcoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFrontRightCANcoder};

  ctre::phoenix6::configs::CANcoderConfiguration wLeftCANcoderConfig{};
  wLeftCANcoderConfig.MagnetSensor.MagnetOffset = CarDriveConstants::kFrontLeftMagnetOffset;
  wLeftCANcoderConfig.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  mFrontLeftCANcoder->GetConfigurator().Apply(wLeftCANcoderConfig);

  ctre::phoenix6::configs::CANcoderConfiguration wRightCANcoderConfig{};
  wRightCANcoderConfig.MagnetSensor.MagnetOffset = CarDriveConstants::kFrontRightMagnetOffset;
  wRightCANcoderConfig.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  mFrontRightCANcoder->GetConfigurator().Apply(wRightCANcoderConfig);

  ctre::phoenix6::configs::TalonFXConfiguration wLeftDriveConfig{};
  wLeftDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  wLeftDriveConfig.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  wLeftDriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  wLeftDriveConfig.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
  wLeftDriveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  wLeftDriveConfig.MotorOutput.Inverted = CarDriveConstants::kFrontLeftDriveInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  mFrontLeftDrive->GetConfigurator().Apply(wLeftDriveConfig);

  ctre::phoenix6::configs::TalonFXConfiguration wRightDriveConfig{};
  wRightDriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  wRightDriveConfig.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  wRightDriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  wRightDriveConfig.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
  wRightDriveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  wRightDriveConfig.MotorOutput.Inverted = CarDriveConstants::kFrontRightDriveInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  mFrontRightDrive->GetConfigurator().Apply(wRightDriveConfig);

  ctre::phoenix6::configs::TalonFXConfiguration wLeftSteerConfig{};
  wLeftSteerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  wLeftSteerConfig.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  wLeftSteerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  wLeftSteerConfig.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
  wLeftSteerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  wLeftSteerConfig.MotorOutput.Inverted = CarDriveConstants::kFrontLeftSteerInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  wLeftSteerConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  wLeftSteerConfig.Feedback.FeedbackRemoteSensorID = CANid::kFrontLeftCANcoder;
  wLeftSteerConfig.Feedback.RotorToSensorRatio = CarDriveConstants::kSteerGearRatio;
  wLeftSteerConfig.Feedback.SensorToMechanismRatio = 1.0;
  wLeftSteerConfig.ClosedLoopGeneral.ContinuousWrap = true;
  wLeftSteerConfig.Slot0.kP = CarDriveConstants::kSteerP;
  wLeftSteerConfig.Slot0.kI = CarDriveConstants::kSteerI;
  wLeftSteerConfig.Slot0.kD = CarDriveConstants::kSteerD;
  mFrontLeftSteer->GetConfigurator().Apply(wLeftSteerConfig);

  ctre::phoenix6::configs::TalonFXConfiguration wRightSteerConfig = wLeftSteerConfig;
  wRightSteerConfig.MotorOutput.Inverted = CarDriveConstants::kFrontRightSteerInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  wRightSteerConfig.Feedback.FeedbackRemoteSensorID = CANid::kFrontRightCANcoder;
  mFrontRightSteer->GetConfigurator().Apply(wRightSteerConfig);
}

SubCarDrive::~SubCarDrive() {
  delete mFrontLeftDrive;
  delete mFrontLeftSteer;
  delete mFrontLeftCANcoder;

  delete mFrontRightDrive;
  delete mFrontRightSteer;
  delete mFrontRightCANcoder;
}

void SubCarDrive::Periodic() {}

void SubCarDrive::drive(double iThrottle, double iBrake, double iSteer) {
  double wClampedThrottle = std::clamp(iThrottle, 0.0, 1.0);
  double wClampedBrake = std::clamp(iBrake, 0.0, 1.0);
  double wNetThrottle = (wClampedThrottle * CarDriveConstants::kSpeedScale) * (1.0 - wClampedBrake);
  if (wNetThrottle < 0.0) {
    wNetThrottle = 0.0;
  }

  units::angle::degree_t wSteerAngle = -std::clamp(iSteer, -1.0, 1.0) * CarDriveConstants::kMaxSteerAngle;
  units::angle::turn_t wSteerTurns = wSteerAngle;

  mFrontLeftDrive->SetControl(mDriveDutyCycleControl.WithOutput(wNetThrottle));
  mFrontRightDrive->SetControl(mDriveDutyCycleControl.WithOutput(wNetThrottle));

  mFrontLeftSteer->SetControl(mSteerPositionControl.WithPosition(wSteerTurns));
  mFrontRightSteer->SetControl(mSteerPositionControl.WithPosition(wSteerTurns));
}

void SubCarDrive::stop() {
  mFrontLeftDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
  mFrontRightDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
}
