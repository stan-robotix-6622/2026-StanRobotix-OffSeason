#pragma once

#include <memory>
#include <string_view>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include "StanMotor.h"
#include "StanSwerveDrivetrain.h"
#include "SwervePresets.h"

namespace stan {

class StanSwerveBuilder {
 public:
  StanSwerveBuilder() = default;

  StanSwerveBuilder& withDimensions(units::length::meter_t iTrackWidth, units::length::meter_t iWheelBase) {
    mTrackWidth = iTrackWidth;
    mWheelBase = iWheelBase;
    return *this;
  }

  StanSwerveBuilder& withPreset(SwervePreset iPreset) {
    mConstants = SwervePresets::get(iPreset);
    mMaxAttainableSpeed = mConstants.kMaxFreeSpeed;
    return *this;
  }

  StanSwerveBuilder& withCustomConstants(const SwerveModuleConstants& iConstants) {
    mConstants = iConstants;
    mMaxAttainableSpeed = mConstants.kMaxFreeSpeed;
    return *this;
  }

  StanSwerveBuilder& withCanBus(std::string_view iCanBus) {
    mCanBus = iCanBus;
    return *this;
  }

  StanSwerveBuilder& withMotorTypes(MotorType iDriveType, MotorType iSteerType) {
    mDriveType = iDriveType;
    mSteerType = iSteerType;
    return *this;
  }

  StanSwerveBuilder& withPigeon2(int iCanId, std::string_view iCanBus = "rio") {
    mPigeonId = iCanId;
    mPigeonBus = iCanBus;
    mHasPigeonId = true;
    return *this;
  }

  StanSwerveBuilder& withPigeon2(ctre::phoenix6::hardware::Pigeon2* iPigeon) {
    mExternalPigeon = iPigeon;
    return *this;
  }

  StanSwerveBuilder& withModuleFL(int iDriveId, int iSteerId, int iEncoderId = -1,
                                 units::angle::turn_t iMagnetOffset = 0_tr,
                                 bool iDriveInverted = false, bool iSteerInverted = false) {
    mFLDriveId = iDriveId;
    mFLSteerId = iSteerId;
    mFLEncoderId = iEncoderId;
    mFLOffset = iMagnetOffset;
    mFLDriveInverted = iDriveInverted;
    mFLSteerInverted = iSteerInverted;
    return *this;
  }

  StanSwerveBuilder& withModuleFR(int iDriveId, int iSteerId, int iEncoderId = -1,
                                 units::angle::turn_t iMagnetOffset = 0_tr,
                                 bool iDriveInverted = false, bool iSteerInverted = false) {
    mFRDriveId = iDriveId;
    mFRSteerId = iSteerId;
    mFREncoderId = iEncoderId;
    mFROffset = iMagnetOffset;
    mFRDriveInverted = iDriveInverted;
    mFRSteerInverted = iSteerInverted;
    return *this;
  }

  StanSwerveBuilder& withModuleBL(int iDriveId, int iSteerId, int iEncoderId = -1,
                                 units::angle::turn_t iMagnetOffset = 0_tr,
                                 bool iDriveInverted = false, bool iSteerInverted = false) {
    mBLDriveId = iDriveId;
    mBLSteerId = iSteerId;
    mBLEncoderId = iEncoderId;
    mBLOffset = iMagnetOffset;
    mBLDriveInverted = iDriveInverted;
    mBLSteerInverted = iSteerInverted;
    return *this;
  }

  StanSwerveBuilder& withModuleBR(int iDriveId, int iSteerId, int iEncoderId = -1,
                                 units::angle::turn_t iMagnetOffset = 0_tr,
                                 bool iDriveInverted = false, bool iSteerInverted = false) {
    mBRDriveId = iDriveId;
    mBRSteerId = iSteerId;
    mBREncoderId = iEncoderId;
    mBROffset = iMagnetOffset;
    mBRDriveInverted = iDriveInverted;
    mBRSteerInverted = iSteerInverted;
    return *this;
  }

  StanSwerveBuilder& withTranslationSlewRate(units::acceleration::meters_per_second_squared_t iRateLimit) {
    mTranslationRateLimit = iRateLimit;
    return *this;
  }

  StanSwerveBuilder& withMaxAttainableSpeed(units::velocity::meters_per_second_t iMaxSpeed) {
    mMaxAttainableSpeed = iMaxSpeed;
    return *this;
  }

  StanSwerveBuilder& withMaxAngularVelocity(units::angular_velocity::radians_per_second_t iMaxOmega) {
    mMaxAngularSpeed = iMaxOmega;
    return *this;
  }

  StanSwerveDrivetrain* build() {
    auto flModule = new StanSwerveModule{
        mFLDriveId, mFLSteerId, mFLEncoderId, mFLOffset,
        mFLDriveInverted, mFLSteerInverted,
        mDriveType, mSteerType, mCanBus, mConstants};

    auto frModule = new StanSwerveModule{
        mFRDriveId, mFRSteerId, mFREncoderId, mFROffset,
        mFRDriveInverted, mFRSteerInverted,
        mDriveType, mSteerType, mCanBus, mConstants};

    auto blModule = new StanSwerveModule{
        mBLDriveId, mBLSteerId, mBLEncoderId, mBLOffset,
        mBLDriveInverted, mBLSteerInverted,
        mDriveType, mSteerType, mCanBus, mConstants};

    auto brModule = new StanSwerveModule{
        mBRDriveId, mBRSteerId, mBREncoderId, mBROffset,
        mBRDriveInverted, mBRSteerInverted,
        mDriveType, mSteerType, mCanBus, mConstants};

    ctre::phoenix6::hardware::Pigeon2* pigeon = mExternalPigeon;
    bool ownsPigeon = false;
    if (pigeon == nullptr && mHasPigeonId) {
      pigeon = new ctre::phoenix6::hardware::Pigeon2{mPigeonId, mPigeonBus};
      ownsPigeon = true;
    }

    return new StanSwerveDrivetrain{
        flModule, frModule, blModule, brModule,
        mTrackWidth, mWheelBase,
        pigeon, ownsPigeon,
        mTranslationRateLimit, mMaxAttainableSpeed, mMaxAngularSpeed};
  }

  std::unique_ptr<StanSwerveDrivetrain> buildUnique() {
    return std::unique_ptr<StanSwerveDrivetrain>{build()};
  }

 private:
  units::length::meter_t mTrackWidth{24.0_in};
  units::length::meter_t mWheelBase{24.0_in};

  SwerveModuleConstants mConstants{SwervePresets::get(SwervePreset::kSDSMK4_L2)};
  std::string_view mCanBus{"rio"};
  MotorType mDriveType{MotorType::kTalonFX};
  MotorType mSteerType{MotorType::kTalonFX};

  int mPigeonId{-1};
  std::string_view mPigeonBus{"rio"};
  bool mHasPigeonId{false};
  ctre::phoenix6::hardware::Pigeon2* mExternalPigeon{nullptr};

  int mFLDriveId{0};
  int mFLSteerId{0};
  int mFLEncoderId{-1};
  units::angle::turn_t mFLOffset{0_tr};
  bool mFLDriveInverted{false};
  bool mFLSteerInverted{false};

  int mFRDriveId{0};
  int mFRSteerId{0};
  int mFREncoderId{-1};
  units::angle::turn_t mFROffset{0_tr};
  bool mFRDriveInverted{false};
  bool mFRSteerInverted{false};

  int mBLDriveId{0};
  int mBLSteerId{0};
  int mBLEncoderId{-1};
  units::angle::turn_t mBLOffset{0_tr};
  bool mBLDriveInverted{false};
  bool mBLSteerInverted{false};

  int mBRDriveId{0};
  int mBRSteerId{0};
  int mBREncoderId{-1};
  units::angle::turn_t mBROffset{0_tr};
  bool mBRDriveInverted{false};
  bool mBRSteerInverted{false};

  units::acceleration::meters_per_second_squared_t mTranslationRateLimit{6.0_mps_sq};
  units::velocity::meters_per_second_t mMaxAttainableSpeed{4.5_mps};
  units::angular_velocity::radians_per_second_t mMaxAngularSpeed{3.0 * std::numbers::pi * 1_rad_per_s};
};

} // namespace stan
