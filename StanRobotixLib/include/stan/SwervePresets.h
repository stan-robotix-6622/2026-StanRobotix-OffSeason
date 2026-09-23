#pragma once

#include <numbers>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

namespace stan {

enum class SwervePreset {
  kSDSMK4_L1,
  kSDSMK4_L2,
  kSDSMK4_L3,
  kSDSMK4_L4,
  kSDSMK4i_L1,
  kSDSMK4i_L2,
  kSDSMK4i_L3,
  kSDSMK4i_L4,
  kREVMAXSwerve
};

struct SwerveModuleConstants {
  double kDriveGearRatio{6.75};
  double kSteerGearRatio{12.8};
  units::length::meter_t kWheelRadius{2.0_in};
  units::velocity::meters_per_second_t kMaxFreeSpeed{4.5_mps};
  units::current::ampere_t kDriveCurrentLimit{40_A};
  units::current::ampere_t kSteerCurrentLimit{20_A};

  constexpr units::length::meter_t getWheelDiameter() const {
    return kWheelRadius * 2.0;
  }

  constexpr units::length::meter_t getWheelCircumference() const {
    return kWheelRadius * 2.0 * std::numbers::pi;
  }
};

class SwervePresets {
 public:
  static constexpr SwerveModuleConstants get(SwervePreset iPreset) {
    switch (iPreset) {
      case SwervePreset::kSDSMK4_L1:
        return {
            8.14,
            12.8,
            2.0_in,
            3.8_mps,
            40_A,
            20_A};
      case SwervePreset::kSDSMK4_L2:
        return {
            6.75,
            12.8,
            2.0_in,
            4.5_mps,
            40_A,
            20_A};
      case SwervePreset::kSDSMK4_L3:
        return {
            6.12,
            12.8,
            2.0_in,
            5.0_mps,
            40_A,
            20_A};
      case SwervePreset::kSDSMK4_L4:
        return {
            5.14,
            12.8,
            2.0_in,
            6.0_mps,
            40_A,
            20_A};
      case SwervePreset::kSDSMK4i_L1:
        return {
            8.14,
            150.0 / 7.0,
            2.0_in,
            3.8_mps,
            40_A,
            20_A};
      case SwervePreset::kSDSMK4i_L2:
        return {
            6.75,
            150.0 / 7.0,
            2.0_in,
            4.5_mps,
            40_A,
            20_A};
      case SwervePreset::kSDSMK4i_L3:
        return {
            6.12,
            150.0 / 7.0,
            2.0_in,
            5.0_mps,
            40_A,
            20_A};
      case SwervePreset::kSDSMK4i_L4:
        return {
            5.14,
            150.0 / 7.0,
            2.0_in,
            6.0_mps,
            40_A,
            20_A};
      case SwervePreset::kREVMAXSwerve:
        return {
            4.71,
            9424.0 / 203.0,
            1.341628_in,
            4.9180_mps,
            40_A,
            20_A};
    }
    return {6.75, 12.8, 2.0_in, 4.5_mps, 40_A, 20_A};
  }
};

} // namespace stan
