#include "subsystems/SubCarDrive.h"

#include <algorithm>
#include <units/angle.h>
#include "Constants.h"

SubCarDrive::SubCarDrive() {
  mFLDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFLDrive};
  mFLSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFLSteer};
  mFLEncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFLEncoder};

  mFRDrive = new ctre::phoenix6::hardware::TalonFX{CANid::kFRDrive};
  mFRSteer = new ctre::phoenix6::hardware::TalonFX{CANid::kFRSteer};
  mFREncoder = new ctre::phoenix6::hardware::CANcoder{CANid::kFREncoder};

  ctre::phoenix6::configs::CANcoderConfiguration cancoderConfig{};
  cancoderConfig.MagnetSensor.MagnetOffset = CarDriveConstants::kFLMagnetOffset;
  cancoderConfig.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
  mFLEncoder->GetConfigurator().Apply(cancoderConfig);

  cancoderConfig.MagnetSensor.MagnetOffset = CarDriveConstants::kFRMagnetOffset;
  mFREncoder->GetConfigurator().Apply(cancoderConfig);

  ctre::phoenix6::configs::TalonFXConfiguration driveConfig{};
  driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  driveConfig.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  driveConfig.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
  driveConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  driveConfig.MotorOutput.Inverted = CarDriveConstants::kFLDriveInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  mFLDrive->GetConfigurator().Apply(driveConfig);

  driveConfig.MotorOutput.Inverted = CarDriveConstants::kFRDriveInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  mFRDrive->GetConfigurator().Apply(driveConfig);

  ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};
  steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
  steerConfig.CurrentLimits.SupplyCurrentLimit = CarDriveConstants::kSupplyCurrentLimit;
  steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  steerConfig.CurrentLimits.StatorCurrentLimit = CarDriveConstants::kStatorCurrentLimit;
  steerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  steerConfig.MotorOutput.Inverted = CarDriveConstants::kFLSteerInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
  steerConfig.Feedback.FeedbackRemoteSensorID = CANid::kFLEncoder;
  steerConfig.Feedback.RotorToSensorRatio = CarDriveConstants::kSteerGearRatio;
  steerConfig.Feedback.SensorToMechanismRatio = 1.0;
  steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
  steerConfig.Slot0.kP = CarDriveConstants::kSteerP;
  steerConfig.Slot0.kI = CarDriveConstants::kSteerI;
  steerConfig.Slot0.kD = CarDriveConstants::kSteerD;
  mFLSteer->GetConfigurator().Apply(steerConfig);

  steerConfig.MotorOutput.Inverted = CarDriveConstants::kFRSteerInverted
      ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
      : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  steerConfig.Feedback.FeedbackRemoteSensorID = CANid::kFREncoder;
  mFRSteer->GetConfigurator().Apply(steerConfig);
}

SubCarDrive::~SubCarDrive() {
  delete mFLDrive;
  delete mFLSteer;
  delete mFLEncoder;

  delete mFRDrive;
  delete mFRSteer;
  delete mFREncoder;
}

void SubCarDrive::Periodic() {}

void SubCarDrive::drive(double iThrottle, double iBrake, double iSteer) {
  double throttle = std::clamp(iThrottle, 0.0, 1.0);
  double brake = std::clamp(iBrake, 0.0, 1.0);
  double speed = (throttle * CarDriveConstants::kSpeedScale) * (1.0 - brake);
  if (speed < 0.0) {
    speed = 0.0;
  }

  units::angle::turn_t steerAngle = -std::clamp(iSteer, -1.0, 1.0) * CarDriveConstants::kMaxSteerAngle;

  mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(speed));
  mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(speed));

  mFLSteer->SetControl(mSteerPositionControl.WithPosition(steerAngle));
  mFRSteer->SetControl(mSteerPositionControl.WithPosition(steerAngle));
}

void SubCarDrive::stop() {
  mFLDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
  mFRDrive->SetControl(mDriveDutyCycleControl.WithOutput(0.0));
}
