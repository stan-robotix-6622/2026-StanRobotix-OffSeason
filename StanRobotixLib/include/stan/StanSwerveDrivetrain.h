#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include <numbers>
#include <optional>
#include <string_view>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/MagnetSensorConfigs.hpp>
#include <ctre/phoenix6/configs/Slot0Configs.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>

#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/math.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/array.h>

#include "KrakenSync.h"
#include "StanMotor.h"
#include "SwervePresets.h"

namespace stan {

class StanSwerveModule {
 public:
  StanSwerveModule(int iDriveMotorId, int iSteerMotorId, int iEncoderId = -1,
                   units::angle::turn_t iMagnetOffset = 0_tr,
                   bool iDriveInverted = false, bool iSteerInverted = false,
                   MotorType iDriveType = MotorType::kTalonFX,
                   MotorType iSteerType = MotorType::kTalonFX,
                   std::string_view iCanBus = "rio",
                   const SwerveModuleConstants& iConstants = SwervePresets::get(SwervePreset::kSDSMK4_L2))
      : mConstants{iConstants},
        mMagnetOffset{iMagnetOffset},
        mDriveInverted{iDriveInverted},
        mSteerInverted{iSteerInverted} {
    mDriveMotor = new StanMotor{iDriveMotorId, iDriveType, iCanBus};
    mSteerMotor = new StanMotor{iSteerMotorId, iSteerType, iCanBus};

    mDriveMotor->setInverted(iDriveInverted);
    mSteerMotor->setInverted(iSteerInverted);

    mDriveMotor->setCurrentLimit(mConstants.kDriveCurrentLimit);
    mSteerMotor->setCurrentLimit(mConstants.kSteerCurrentLimit);

    mDriveMotor->setIdleMode(IdleMode::kBrake);
    mSteerMotor->setIdleMode(IdleMode::kBrake);

    if (iEncoderId >= 0) {
      mCANcoder = new ctre::phoenix6::hardware::CANcoder{iEncoderId, iCanBus};
      ctre::phoenix6::configs::CANcoderConfiguration cancoderConfig{};
      cancoderConfig.MagnetSensor.MagnetOffset = iMagnetOffset;
      cancoderConfig.MagnetSensor.SensorDirection =
          ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
      mCANcoder->GetConfigurator().Apply(cancoderConfig);
    }

    if (auto steerTalon = mSteerMotor->getTalonFX()) {
      ctre::phoenix6::configs::TalonFXConfiguration steerConfig{};
      steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      steerConfig.CurrentLimits.SupplyCurrentLimit = mConstants.kSteerCurrentLimit;
      steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      steerConfig.CurrentLimits.StatorCurrentLimit = 40_A;
      steerConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
      steerConfig.MotorOutput.Inverted = iSteerInverted
          ? ctre::phoenix6::signals::InvertedValue::Clockwise_Positive
          : ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
      steerConfig.Feedback.FeedbackSensorSource =
          ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
      steerConfig.Feedback.SensorToMechanismRatio = mConstants.kSteerGearRatio;
      steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
      steerConfig.Slot0.kP = 40.0;
      steerConfig.Slot0.kD = 0.5;
      steerTalon->GetConfigurator().Apply(steerConfig);

      if (mCANcoder != nullptr) {
        KrakenSync::sync(steerTalon, mCANcoder, 250_ms);
      }
    }
  }

  ~StanSwerveModule() {
    delete mDriveMotor;
    delete mSteerMotor;
    delete mCANcoder;
  }

  void setDesiredState(const frc::SwerveModuleState& iDesiredState, bool iIsOpenLoop = false) {
    mCurrentRotation2d = getRotation2d();
    mOptimizedState = iDesiredState;
    mOptimizedState.Optimize(mCurrentRotation2d);

    if (auto steerTalon = mSteerMotor->getTalonFX()) {
      steerTalon->SetControl(mSteerPositionControl.WithPosition(units::angle::turn_t{mOptimizedState.angle.Radians()}));
    } else {
      mSteerMotor->setPosition(units::angle::turn_t{mOptimizedState.angle.Radians() * mConstants.kSteerGearRatio});
    }

    if (iIsOpenLoop) {
      double speedFraction = mOptimizedState.speed / mConstants.kMaxFreeSpeed;
      mDriveMotor->set(std::clamp(speedFraction, -1.0, 1.0));
    } else {
      double wheelRPS = mOptimizedState.speed.value() / mConstants.getWheelCircumference().value();
      units::angular_velocity::turns_per_second_t motorVelocity{wheelRPS * mConstants.kDriveGearRatio};
      mDriveMotor->setVelocity(motorVelocity);
    }
  }

  void setDesiredHeading(const frc::Rotation2d& iHeading) {
    setDesiredState(frc::SwerveModuleState{0_mps, iHeading}, true);
  }

  frc::SwerveModulePosition getModulePosition() {
    units::length::meter_t distance = (mDriveMotor->getPosition().value() / mConstants.kDriveGearRatio) * mConstants.getWheelCircumference();
    mCurrentPosition.distance = distance;
    mCurrentPosition.angle = getRotation2d();
    return mCurrentPosition;
  }

  frc::SwerveModuleState getModuleState() {
    double wheelRPS = mDriveMotor->getVelocity().value() / mConstants.kDriveGearRatio;
    units::velocity::meters_per_second_t speed{wheelRPS * mConstants.getWheelCircumference().value()};
    mCurrentState.speed = speed;
    mCurrentState.angle = getRotation2d();
    return mCurrentState;
  }

  frc::Rotation2d getRotation2d() const {
    if (auto steerTalon = mSteerMotor->getTalonFX()) {
      units::angle::turn_t mechanismTurns = steerTalon->GetPosition().GetValue();
      return frc::Rotation2d{units::angle::radian_t{mechanismTurns}};
    }
    return frc::Rotation2d{units::angle::radian_t{mSteerMotor->getPosition() / mConstants.kSteerGearRatio}};
  }

  void stop() {
    mDriveMotor->set(0.0);
  }

  void setIdleMode(IdleMode iMode) {
    mDriveMotor->setIdleMode(iMode);
    mSteerMotor->setIdleMode(iMode);
  }

  void syncToCANcoder() {
    if (mCANcoder != nullptr && mSteerMotor->getTalonFX() != nullptr) {
      KrakenSync::sync(mSteerMotor->getTalonFX(), mCANcoder, 250_ms);
    }
  }

  StanMotor* getDriveMotor() { return mDriveMotor; }
  StanMotor* getSteerMotor() { return mSteerMotor; }
  ctre::phoenix6::hardware::CANcoder* getCANcoder() { return mCANcoder; }
  const SwerveModuleConstants& getConstants() const { return mConstants; }

 private:
  StanMotor* mDriveMotor{nullptr};
  StanMotor* mSteerMotor{nullptr};
  ctre::phoenix6::hardware::CANcoder* mCANcoder{nullptr};

  SwerveModuleConstants mConstants;
  units::angle::turn_t mMagnetOffset{0_tr};
  bool mDriveInverted{false};
  bool mSteerInverted{false};

  ctre::phoenix6::controls::PositionVoltage mSteerPositionControl{0_tr};

  frc::SwerveModuleState mOptimizedState{};
  frc::SwerveModuleState mCurrentState{};
  frc::SwerveModulePosition mCurrentPosition{};
  frc::Rotation2d mCurrentRotation2d{0_rad};
};

class StanSwerveDrivetrain : public frc2::SubsystemBase {
 public:
  StanSwerveDrivetrain(
      StanSwerveModule* iFrontLeft,
      StanSwerveModule* iFrontRight,
      StanSwerveModule* iBackLeft,
      StanSwerveModule* iBackRight,
      units::length::meter_t iTrackWidth,
      units::length::meter_t iWheelBase,
      ctre::phoenix6::hardware::Pigeon2* iPigeon = nullptr,
      bool iOwnsPigeon = false,
      units::acceleration::meters_per_second_squared_t iTranslationRateLimit = 6.0_mps_sq,
      units::velocity::meters_per_second_t iMaxAttainableSpeed = 4.5_mps,
      units::angular_velocity::radians_per_second_t iMaxAngularSpeed = 3.0 * std::numbers::pi * 1_rad_per_s)
      : mFrontLeftModule{iFrontLeft},
        mFrontRightModule{iFrontRight},
        mBackLeftModule{iBackLeft},
        mBackRightModule{iBackRight},
        mPigeon{iPigeon},
        mOwnsPigeon{iOwnsPigeon},
        mFrontLeftLocation{+iWheelBase / 2.0, +iTrackWidth / 2.0},
        mFrontRightLocation{+iWheelBase / 2.0, -iTrackWidth / 2.0},
        mBackLeftLocation{-iWheelBase / 2.0, +iTrackWidth / 2.0},
        mBackRightLocation{-iWheelBase / 2.0, -iTrackWidth / 2.0},
        mXLimiter{iTranslationRateLimit},
        mYLimiter{iTranslationRateLimit},
        mMaxAttainableSpeed{iMaxAttainableSpeed},
        mMaxDesiredSpeed{iMaxAttainableSpeed},
        mMaxDesiredAngularSpeed{iMaxAngularSpeed} {
    mKinematics = new frc::SwerveDriveKinematics<4>{
        mFrontLeftLocation,
        mFrontRightLocation,
        mBackLeftLocation,
        mBackRightLocation};

    mCurrentRotation2d = getRotation2d();
    mCurrentModulePositions = getModulePositions();

    mPoseEstimator = new frc::SwerveDrivePoseEstimator<4>{
        *mKinematics,
        mCurrentRotation2d,
        mCurrentModulePositions,
        frc::Pose2d{0_m, 0_m, 0_deg},
        {0.1, 0.1, 0.1},
        {0.7, 0.7, 999999.0}};
  }

  ~StanSwerveDrivetrain() override {
    delete mFrontLeftModule;
    delete mFrontRightModule;
    delete mBackLeftModule;
    delete mBackRightModule;
    delete mKinematics;
    delete mPoseEstimator;
    if (mOwnsPigeon) {
      delete mPigeon;
    }
  }

  void Periodic() override {
    mCurrentRotation2d = getRotation2d();
    mCurrentModulePositions = getModulePositions();
    mPoseEstimator->Update(mCurrentRotation2d, mCurrentModulePositions);
    mCurrentPose = mPoseEstimator->GetEstimatedPosition();
    updateTelemetry();
  }

  void drive(units::velocity::meters_per_second_t iVx,
             units::velocity::meters_per_second_t iVy,
             units::angular_velocity::radians_per_second_t iOmega,
             bool iFieldRelative = true) {
    units::velocity::meters_per_second_t limitedVx = mXLimiter.Calculate(iVx);
    units::velocity::meters_per_second_t limitedVy = mYLimiter.Calculate(iVy);

    if (iFieldRelative) {
      mDesiredChassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
          limitedVx, limitedVy, iOmega, getRotation2d());
    } else {
      mDesiredChassisSpeeds = frc::ChassisSpeeds{limitedVx, limitedVy, iOmega};
    }

    mDesiredChassisSpeeds = frc::ChassisSpeeds::Discretize(mDesiredChassisSpeeds, 0.02_s);
    mDesiredModuleStates = mKinematics->ToSwerveModuleStates(mDesiredChassisSpeeds);
    mKinematics->DesaturateWheelSpeeds(&mDesiredModuleStates, mMaxAttainableSpeed);

    setModuleStates(mDesiredModuleStates);
  }

  void drive(double iXInput, double iYInput, double iRotInput, bool iFieldRelative = true) {
    units::velocity::meters_per_second_t vx = std::clamp(iXInput, -1.0, 1.0) * mMaxDesiredSpeed;
    units::velocity::meters_per_second_t vy = std::clamp(iYInput, -1.0, 1.0) * mMaxDesiredSpeed;
    units::angular_velocity::radians_per_second_t omega =
        std::clamp(iRotInput, -1.0, 1.0) * mMaxDesiredAngularSpeed;
    drive(vx, vy, omega, iFieldRelative);
  }

  void driveRobotRelative(const frc::ChassisSpeeds& iSpeeds) {
    mDesiredChassisSpeeds = frc::ChassisSpeeds::Discretize(iSpeeds, 0.02_s);
    mDesiredModuleStates = mKinematics->ToSwerveModuleStates(mDesiredChassisSpeeds);
    mKinematics->DesaturateWheelSpeeds(&mDesiredModuleStates, mMaxAttainableSpeed);
    setModuleStates(mDesiredModuleStates);
  }

  void setModuleStates(const wpi::array<frc::SwerveModuleState, 4>& iStates, bool iIsOpenLoop = false) {
    mFrontLeftModule->setDesiredState(iStates[0], iIsOpenLoop);
    mFrontRightModule->setDesiredState(iStates[1], iIsOpenLoop);
    mBackLeftModule->setDesiredState(iStates[2], iIsOpenLoop);
    mBackRightModule->setDesiredState(iStates[3], iIsOpenLoop);
  }

  void setXFormation() {
    mFrontLeftModule->setDesiredHeading(45_deg);
    mFrontRightModule->setDesiredHeading(135_deg);
    mBackLeftModule->setDesiredHeading(135_deg);
    mBackRightModule->setDesiredHeading(45_deg);
  }

  void stop() {
    mFrontLeftModule->stop();
    mFrontRightModule->stop();
    mBackLeftModule->stop();
    mBackRightModule->stop();
  }

  void setIdleMode(IdleMode iMode) {
    mFrontLeftModule->setIdleMode(iMode);
    mFrontRightModule->setIdleMode(iMode);
    mBackLeftModule->setIdleMode(iMode);
    mBackRightModule->setIdleMode(iMode);
  }

  void setBrakeMode(bool iBrake) {
    setIdleMode(iBrake ? IdleMode::kBrake : IdleMode::kCoast);
  }

  void addVisionMeasurement(
      const frc::Pose2d& iPose,
      units::time::second_t iTimestampSeconds,
      units::angular_velocity::degrees_per_second_t iRotationalVelocity,
      int iTagCount,
      const wpi::array<double, 3>& iVisionStdDevs = {0.7, 0.7, 999999.0}) {
    if (units::math::abs(iRotationalVelocity) > 180_deg_per_s) {
      return;
    }
    if (iTagCount == 0) {
      return;
    }
    if (iPose.X() == 0_m && iPose.Y() == 0_m && iPose.Rotation().Radians() == 0_rad) {
      return;
    }

    mPoseEstimator->SetVisionMeasurementStdDevs(iVisionStdDevs);
    mPoseEstimator->AddVisionMeasurement(iPose, iTimestampSeconds);
  }

  void addVisionMeasurement(
      const frc::Pose2d& iPose,
      units::time::second_t iTimestampSeconds,
      const wpi::array<double, 3>& iVisionStdDevs = {0.7, 0.7, 999999.0}) {
    if (iPose.X() == 0_m && iPose.Y() == 0_m && iPose.Rotation().Radians() == 0_rad) {
      return;
    }
    units::angular_velocity::degrees_per_second_t rotVel = getRotationalVelocity();
    if (units::math::abs(rotVel) > 180_deg_per_s) {
      return;
    }

    mPoseEstimator->SetVisionMeasurementStdDevs(iVisionStdDevs);
    mPoseEstimator->AddVisionMeasurement(iPose, iTimestampSeconds);
  }

  frc::Pose2d getPose() const {
    return mCurrentPose;
  }

  void resetPose(const frc::Pose2d& iPose) {
    mPoseEstimator->ResetPosition(getRotation2d(), getModulePositions(), iPose);
    mCurrentPose = iPose;
  }

  void resetHeading(units::angle::degree_t iAngle = 0_deg) {
    if (mPigeon != nullptr) {
      mPigeon->SetYaw(iAngle);
    }
    mPoseEstimator->ResetRotation(iAngle);
  }

  frc::Rotation2d getRotation2d() const {
    if (mPigeon != nullptr) {
      return mPigeon->GetRotation2d();
    }
    return frc::Rotation2d{0_deg};
  }

  units::angular_velocity::degrees_per_second_t getRotationalVelocity() const {
    if (mPigeon != nullptr) {
      return mPigeon->GetAngularVelocityZWorld().GetValue();
    }
    return 0_deg_per_s;
  }

  wpi::array<frc::SwerveModulePosition, 4> getModulePositions() {
    mCurrentModulePositions[0] = mFrontLeftModule->getModulePosition();
    mCurrentModulePositions[1] = mFrontRightModule->getModulePosition();
    mCurrentModulePositions[2] = mBackLeftModule->getModulePosition();
    mCurrentModulePositions[3] = mBackRightModule->getModulePosition();
    return mCurrentModulePositions;
  }

  wpi::array<frc::SwerveModuleState, 4> getModuleStates() {
    mCurrentModuleStates[0] = mFrontLeftModule->getModuleState();
    mCurrentModuleStates[1] = mFrontRightModule->getModuleState();
    mCurrentModuleStates[2] = mBackLeftModule->getModuleState();
    mCurrentModuleStates[3] = mBackRightModule->getModuleState();
    return mCurrentModuleStates;
  }

  frc::ChassisSpeeds getRobotRelativeSpeeds() {
    mCurrentChassisSpeeds = mKinematics->ToChassisSpeeds(getModuleStates());
    return mCurrentChassisSpeeds;
  }

  frc::ChassisSpeeds getFieldRelativeSpeeds() {
    frc::ChassisSpeeds robotSpeeds = getRobotRelativeSpeeds();
    return frc::ChassisSpeeds::FromRobotRelativeSpeeds(
        robotSpeeds.vx, robotSpeeds.vy, robotSpeeds.omega, getRotation2d());
  }

  void syncSteerEncoders() {
    mFrontLeftModule->syncToCANcoder();
    mFrontRightModule->syncToCANcoder();
    mBackLeftModule->syncToCANcoder();
    mBackRightModule->syncToCANcoder();
  }

  frc2::CommandPtr getDriveCommand(
      std::function<double()> iXSupplier,
      std::function<double()> iYSupplier,
      std::function<double()> iRotSupplier,
      bool iFieldRelative = true) {
    return Run([this, iXSupplier, iYSupplier, iRotSupplier, iFieldRelative] {
      drive(iXSupplier(), iYSupplier(), iRotSupplier(), iFieldRelative);
    });
  }

  frc2::CommandPtr getStopCommand() {
    return RunOnce([this] { stop(); });
  }

  frc2::CommandPtr getXFormationCommand() {
    return Run([this] { setXFormation(); });
  }

  frc2::CommandPtr getZeroHeadingCommand() {
    return RunOnce([this] { resetHeading(0_deg); });
  }

  frc2::CommandPtr getBrakeModeCommand(bool iBrake) {
    return RunOnce([this, iBrake] { setBrakeMode(iBrake); });
  }

  frc2::CommandPtr getResetPoseCommand(const frc::Pose2d& iPose) {
    return RunOnce([this, iPose] { resetPose(iPose); });
  }

  StanSwerveModule* getFLModule() { return mFrontLeftModule; }
  StanSwerveModule* getFRModule() { return mFrontRightModule; }
  StanSwerveModule* getBLModule() { return mBackLeftModule; }
  StanSwerveModule* getBRModule() { return mBackRightModule; }
  ctre::phoenix6::hardware::Pigeon2* getPigeon2() { return mPigeon; }
  frc::SwerveDriveKinematics<4>* getKinematics() { return mKinematics; }
  frc::SwerveDrivePoseEstimator<4>* getPoseEstimator() { return mPoseEstimator; }

 private:
  void updateTelemetry() {
    mCurrentPose2d = mCurrentPose;
  }

  StanSwerveModule* mFrontLeftModule{nullptr};
  StanSwerveModule* mFrontRightModule{nullptr};
  StanSwerveModule* mBackLeftModule{nullptr};
  StanSwerveModule* mBackRightModule{nullptr};

  ctre::phoenix6::hardware::Pigeon2* mPigeon{nullptr};
  bool mOwnsPigeon{false};

  frc::Translation2d mFrontLeftLocation;
  frc::Translation2d mFrontRightLocation;
  frc::Translation2d mBackLeftLocation;
  frc::Translation2d mBackRightLocation;

  frc::SwerveDriveKinematics<4>* mKinematics{nullptr};
  frc::SwerveDrivePoseEstimator<4>* mPoseEstimator{nullptr};

  frc::SlewRateLimiter<units::meters_per_second> mXLimiter;
  frc::SlewRateLimiter<units::meters_per_second> mYLimiter;

  units::velocity::meters_per_second_t mMaxAttainableSpeed;
  units::velocity::meters_per_second_t mMaxDesiredSpeed;
  units::angular_velocity::radians_per_second_t mMaxDesiredAngularSpeed;

  frc::Pose2d mCurrentPose{0_m, 0_m, 0_rad};
  frc::Pose2d mCurrentPose2d{0_m, 0_m, 0_rad};
  frc::ChassisSpeeds mDesiredChassisSpeeds{};
  frc::ChassisSpeeds mCurrentChassisSpeeds{};
  frc::Rotation2d mCurrentRotation2d{0_rad};

  wpi::array<frc::SwerveModuleState, 4> mDesiredModuleStates{
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}},
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}},
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}},
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}}};
  wpi::array<frc::SwerveModuleState, 4> mCurrentModuleStates{
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}},
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}},
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}},
      frc::SwerveModuleState{0_mps, frc::Rotation2d{0_rad}}};
  wpi::array<frc::SwerveModulePosition, 4> mCurrentModulePositions{
      frc::SwerveModulePosition{0_m, frc::Rotation2d{0_rad}},
      frc::SwerveModulePosition{0_m, frc::Rotation2d{0_rad}},
      frc::SwerveModulePosition{0_m, frc::Rotation2d{0_rad}},
      frc::SwerveModulePosition{0_m, frc::Rotation2d{0_rad}}};
};

} // namespace stan
