#include <cmath>
#include <numbers>
#include <gtest/gtest.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include "stan/StanCarDrive.h"
#include "stan/SwervePresets.h"
#include "test_helpers.h"

using namespace stan;
using namespace stan::test;

// ============================================================================
// Tier 1: Feature Coverage (Features 10, 11, 12, 13, 14, 15: 30 Tests Total)
// ============================================================================

// --- Feature 10: Swerve Presets & Geometry ---
TEST(KinematicsTier1, PresetSDSMK4_L1) {
  auto p = SwervePresets::get(SwervePreset::kSDSMK4_L1);
  EXPECT_DOUBLE_EQ(p.kDriveGearRatio, 8.14);
  EXPECT_DOUBLE_EQ(p.kSteerGearRatio, 12.8);
  EXPECT_DOUBLE_EQ(p.kWheelRadius.value(), 0.0508);
  EXPECT_DOUBLE_EQ(p.kMaxFreeSpeed.value(), 3.8);
}

TEST(KinematicsTier1, PresetSDSMK4_L2) {
  auto p = SwervePresets::get(SwervePreset::kSDSMK4_L2);
  EXPECT_DOUBLE_EQ(p.kDriveGearRatio, 6.75);
  EXPECT_DOUBLE_EQ(p.kSteerGearRatio, 12.8);
  EXPECT_DOUBLE_EQ(p.kMaxFreeSpeed.value(), 4.5);
}

TEST(KinematicsTier1, PresetSDSMK4_L3) {
  auto p = SwervePresets::get(SwervePreset::kSDSMK4_L3);
  EXPECT_DOUBLE_EQ(p.kDriveGearRatio, 6.12);
  EXPECT_DOUBLE_EQ(p.kSteerGearRatio, 12.8);
  EXPECT_DOUBLE_EQ(p.kMaxFreeSpeed.value(), 5.0);
}

TEST(KinematicsTier1, PresetSDSMK4_L4) {
  auto p = SwervePresets::get(SwervePreset::kSDSMK4_L4);
  EXPECT_DOUBLE_EQ(p.kDriveGearRatio, 5.14);
  EXPECT_DOUBLE_EQ(p.kSteerGearRatio, 12.8);
  EXPECT_DOUBLE_EQ(p.kMaxFreeSpeed.value(), 6.0);
}

TEST(KinematicsTier1, PresetREVMAXSwerve) {
  auto p = SwervePresets::get(SwervePreset::kREVMAXSwerve);
  EXPECT_DOUBLE_EQ(p.kDriveGearRatio, 4.71);
  EXPECT_NEAR(p.kSteerGearRatio, 9424.0 / 203.0, 1e-4);
  EXPECT_NEAR(p.kWheelRadius.value(), 0.03407735, 1e-5);
  EXPECT_DOUBLE_EQ(p.kMaxFreeSpeed.value(), 4.9180);
}

// --- Feature 11: Speed Discretization ---
TEST(KinematicsTier1, DiscretizeZeroSpeeds) {
  frc::ChassisSpeeds zeroSpeeds{0.0_mps, 0.0_mps, 0.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(zeroSpeeds, 0.02_s);
  EXPECT_DOUBLE_EQ(discretized.vx.value(), 0.0);
  EXPECT_DOUBLE_EQ(discretized.vy.value(), 0.0);
  EXPECT_DOUBLE_EQ(discretized.omega.value(), 0.0);
}

TEST(KinematicsTier1, DiscretizePureTranslationForward) {
  frc::ChassisSpeeds translationOnly{3.0_mps, 0.0_mps, 0.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(translationOnly, 0.02_s);
  EXPECT_NEAR(discretized.vx.value(), 3.0, 1e-6);
  EXPECT_NEAR(discretized.vy.value(), 0.0, 1e-6);
  EXPECT_DOUBLE_EQ(discretized.omega.value(), 0.0);
}

TEST(KinematicsTier1, DiscretizePureTranslationSideways) {
  frc::ChassisSpeeds strafeOnly{0.0_mps, 2.5_mps, 0.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(strafeOnly, 0.02_s);
  EXPECT_NEAR(discretized.vx.value(), 0.0, 1e-6);
  EXPECT_NEAR(discretized.vy.value(), 2.5, 1e-6);
  EXPECT_DOUBLE_EQ(discretized.omega.value(), 0.0);
}

TEST(KinematicsTier1, DiscretizeSimultaneousTranslationAndPositiveRotation) {
  frc::ChassisSpeeds combinedSpeeds{3.0_mps, 0.0_mps, 3.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(combinedSpeeds, 0.02_s);
  EXPECT_NE(discretized.vy.value(), 0.0);
  EXPECT_LT(discretized.vy.value(), 0.0);
  EXPECT_NEAR(discretized.omega.value(), 3.0, 1e-6);
}

TEST(KinematicsTier1, DiscretizeSimultaneousTranslationAndNegativeRotation) {
  frc::ChassisSpeeds combinedSpeeds{3.0_mps, 0.0_mps, -3.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(combinedSpeeds, 0.02_s);
  EXPECT_NE(discretized.vy.value(), 0.0);
  EXPECT_GT(discretized.vy.value(), 0.0);
  EXPECT_NEAR(discretized.omega.value(), -3.0, 1e-6);
}

// --- Feature 12: Driver Translation Slew Rate Limiter ---
TEST(KinematicsTier1, SlewRateLimiterRateLimitingStep) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{2.0}};
  limiter.reset(0.0_mps);

  auto step1 = limiter.calculate(5.0_mps, 0.02_s);
  EXPECT_NEAR(step1.value(), 0.04, 1e-6);

  auto step2 = limiter.calculate(5.0_mps, 0.02_s);
  EXPECT_NEAR(step2.value(), 0.08, 1e-6);

  auto step3 = limiter.calculate(5.0_mps, 0.02_s);
  EXPECT_NEAR(step3.value(), 0.12, 1e-6);
}

TEST(KinematicsTier1, SlewRateLimiterSteadyStateReached) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{10.0}};
  limiter.reset(2.0_mps);

  auto step = limiter.calculate(2.0_mps, 0.02_s);
  EXPECT_NEAR(step.value(), 2.0, 1e-6);
}

TEST(KinematicsTier1, SlewRateLimiterDecelerationProfile) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{2.0}};
  limiter.reset(3.0_mps);

  auto step = limiter.calculate(0.0_mps, 0.02_s);
  EXPECT_NEAR(step.value(), 3.0 - 0.04, 1e-6);
}

TEST(KinematicsTier1, SlewRateLimiterZeroRateLimitZeroAdvance) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{0.0}};
  limiter.reset(1.0_mps);

  auto step = limiter.calculate(5.0_mps, 0.02_s);
  EXPECT_NEAR(step.value(), 1.0, 1e-6);
}

TEST(KinematicsTier1, SlewRateLimiterResetPreservesTarget) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{3.0}};
  limiter.reset(1.5_mps);

  auto step = limiter.calculate(1.5_mps, 0.02_s);
  EXPECT_NEAR(step.value(), 1.5, 1e-6);
}

// --- Feature 13: X-Pattern Defensive Lock Math ---
TEST(KinematicsTier1, XPatternLockAnglesAndSpeeds) {
  auto states = calculateXPatternLock();
  EXPECT_EQ(states.size(), 4u);

  for (const auto& state : states) {
    EXPECT_DOUBLE_EQ(state.speed.value(), 0.0);
  }

  EXPECT_NEAR(states[0].angle.Degrees().value(), 45.0, 1e-6);
  EXPECT_NEAR(states[1].angle.Degrees().value(), -45.0, 1e-6);
  EXPECT_NEAR(states[2].angle.Degrees().value(), -45.0, 1e-6);
  EXPECT_NEAR(states[3].angle.Degrees().value(), 45.0, 1e-6);
}

TEST(KinematicsTier1, XPatternOpposingDiagonals) {
  auto states = calculateXPatternLock();
  EXPECT_NEAR(states[0].angle.Degrees().value(), -states[1].angle.Degrees().value(), 1e-6);
  EXPECT_NEAR(states[2].angle.Degrees().value(), -states[3].angle.Degrees().value(), 1e-6);
}

TEST(KinematicsTier1, XPatternAllWheelSpeedsLockedZero) {
  auto states = calculateXPatternLock();
  EXPECT_EQ(states[0].speed, 0.0_mps);
  EXPECT_EQ(states[1].speed, 0.0_mps);
  EXPECT_EQ(states[2].speed, 0.0_mps);
  EXPECT_EQ(states[3].speed, 0.0_mps);
}

TEST(KinematicsTier1, XPatternRotationDegreesCheck) {
  auto states = calculateXPatternLock();
  EXPECT_DOUBLE_EQ(states[0].angle.Degrees().value(), 45.0);
  EXPECT_DOUBLE_EQ(states[1].angle.Degrees().value(), -45.0);
}

TEST(KinematicsTier1, XPatternPerpendicularAxesPairs) {
  auto states = calculateXPatternLock();
  double angleDiff = std::abs(states[0].angle.Degrees().value() - states[1].angle.Degrees().value());
  EXPECT_NEAR(angleDiff, 90.0, 1e-6);
}

// --- Feature 14: Vision Measurement Rejection Filter ---
TEST(KinematicsTier1, VisionFilterAcceptsValidMeasurement) {
  VisionPoseEstimate estimate{
      frc::Pose2d(2.5_m, 1.8_m, frc::Rotation2d(30.0_deg)),
      1.25_s,
      2,
      true};
  bool accepted = evaluateVisionMeasurement(estimate, 45.0_deg_per_s, false);
  EXPECT_TRUE(accepted);
}

TEST(KinematicsTier1, VisionFilterRejectsExcessiveRotationalVelocity) {
  VisionPoseEstimate estimate{
      frc::Pose2d(2.5_m, 1.8_m, frc::Rotation2d(30.0_deg)),
      1.25_s,
      3,
      true};
  bool accepted = evaluateVisionMeasurement(estimate, 185.0_deg_per_s, false);
  EXPECT_FALSE(accepted);

  bool acceptedNeg = evaluateVisionMeasurement(estimate, -190.0_deg_per_s, false);
  EXPECT_FALSE(acceptedNeg);
}

TEST(KinematicsTier1, VisionFilterRejectsZeroTags) {
  VisionPoseEstimate estimate{
      frc::Pose2d(2.5_m, 1.8_m, frc::Rotation2d(30.0_deg)),
      1.25_s,
      0,
      true};
  bool accepted = evaluateVisionMeasurement(estimate, 10.0_deg_per_s, true);
  EXPECT_FALSE(accepted);
}

TEST(KinematicsTier1, VisionFilterRejectsFieldOrigin) {
  VisionPoseEstimate estimate{
      frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(0.0_deg)),
      1.25_s,
      2,
      true};
  bool accepted = evaluateVisionMeasurement(estimate, 10.0_deg_per_s, true);
  EXPECT_FALSE(accepted);
}

TEST(KinematicsTier1, VisionFilterMegaTag1RejectsSingleTag) {
  VisionPoseEstimate estimate{
      frc::Pose2d(3.0_m, 2.0_m, frc::Rotation2d(45.0_deg)),
      1.0_s,
      1,
      true};
  bool acceptedMT1 = evaluateVisionMeasurement(estimate, 10.0_deg_per_s, false);
  EXPECT_FALSE(acceptedMT1);

  bool acceptedMT2 = evaluateVisionMeasurement(estimate, 10.0_deg_per_s, true);
  EXPECT_TRUE(acceptedMT2);
}

// --- Feature 15: CarDrive Kinematics & Steer Clamp ---
TEST(KinematicsTier1, CarDriveThrottleAndBrakeBlending) {
  auto out = calculateCarDrive(1.0, 0.0, 0.0, false, 0.5);
  EXPECT_DOUBLE_EQ(out.mDriveSpeed, 0.5);

  auto halfBrake = calculateCarDrive(1.0, 0.5, 0.0, false, 0.5);
  EXPECT_DOUBLE_EQ(halfBrake.mDriveSpeed, 0.25);

  auto fullBrake = calculateCarDrive(1.0, 1.0, 0.0, false, 0.5);
  EXPECT_DOUBLE_EQ(fullBrake.mDriveSpeed, 0.0);
}

TEST(KinematicsTier1, CarDriveReverseFlagInvertsSpeed) {
  auto forward = calculateCarDrive(0.8, 0.0, 0.0, false, 0.5);
  auto reverse = calculateCarDrive(0.8, 0.0, 0.0, true, 0.5);
  EXPECT_DOUBLE_EQ(forward.mDriveSpeed, 0.4);
  EXPECT_DOUBLE_EQ(reverse.mDriveSpeed, -0.4);
}

TEST(KinematicsTier1, CarDriveSteeringAngleClamping) {
  auto center = calculateCarDrive(0.5, 0.0, 0.0, false);
  EXPECT_DOUBLE_EQ(center.mSteerAngle.value(), 0.0);

  auto fullRight = calculateCarDrive(0.5, 0.0, 1.0, false);
  EXPECT_NEAR(fullRight.mSteerAngle.value(), -60.0, 1e-6);

  auto fullLeft = calculateCarDrive(0.5, 0.0, -1.0, false);
  EXPECT_NEAR(fullLeft.mSteerAngle.value(), 60.0, 1e-6);
}

TEST(KinematicsTier1, CarDriveZeroThrottleZeroSpeed) {
  auto stopped = calculateCarDrive(0.0, 0.0, 0.0, false, 0.5);
  EXPECT_DOUBLE_EQ(stopped.mDriveSpeed, 0.0);
}

TEST(KinematicsTier1, CarDrivePartialThrottleQuarterSteer) {
  auto partial = calculateCarDrive(0.5, 0.0, 0.5, false, 0.5);
  EXPECT_NEAR(partial.mDriveSpeed, 0.25, 1e-6);
  EXPECT_NEAR(partial.mSteerAngle.value(), -30.0, 1e-6);
}

// ============================================================================
// Tier 2: Boundary & Corner Cases (Features 10, 11, 12, 13, 14, 15: 30 Tests Total)
// ============================================================================

// --- Feature 10 Boundary Cases ---
TEST(KinematicsTier2, SDSMK4iPresetGearRatios) {
  auto p = SwervePresets::get(SwervePreset::kSDSMK4i_L2);
  EXPECT_DOUBLE_EQ(p.kDriveGearRatio, 6.75);
  EXPECT_NEAR(p.kSteerGearRatio, 150.0 / 7.0, 1e-6);
  EXPECT_DOUBLE_EQ(p.kDriveCurrentLimit.value(), 40.0);
  EXPECT_DOUBLE_EQ(p.kSteerCurrentLimit.value(), 20.0);
}

TEST(KinematicsTier2, PresetWheelCircumferenceCalculation) {
  auto p = SwervePresets::get(SwervePreset::kSDSMK4_L1);
  EXPECT_NEAR(p.getWheelDiameter().value(), 0.1016, 1e-6);
  EXPECT_NEAR(p.getWheelCircumference().value(), 0.1016 * std::numbers::pi, 1e-6);
}

TEST(KinematicsTier2, PresetMaxFreeSpeedBoundary) {
  auto l4 = SwervePresets::get(SwervePreset::kSDSMK4_L4);
  EXPECT_GT(l4.kMaxFreeSpeed.value(), 5.5);
  EXPECT_LE(l4.kMaxFreeSpeed.value(), 6.5);
}

TEST(KinematicsTier2, PresetSteerCurrentLimitSafety) {
  auto p = SwervePresets::get(SwervePreset::kREVMAXSwerve);
  EXPECT_DOUBLE_EQ(p.kSteerCurrentLimit.value(), 20.0);
  EXPECT_DOUBLE_EQ(p.kDriveCurrentLimit.value(), 40.0);
}

TEST(KinematicsTier2, PresetWheelRadiusNonZero) {
  for (int idx = 0; idx <= 8; ++idx) {
    auto p = SwervePresets::get(static_cast<SwervePreset>(idx));
    EXPECT_GT(p.kWheelRadius.value(), 0.0);
    EXPECT_GT(p.kDriveGearRatio, 0.0);
  }
}

// --- Feature 11 Boundary Cases ---
TEST(KinematicsTier2, DiscretizeExtremeRotationRate) {
  frc::ChassisSpeeds extreme{4.0_mps, 0.0_mps, 720.0_deg_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(extreme, 0.02_s);
  EXPECT_FALSE(std::isnan(discretized.vx.value()));
  EXPECT_FALSE(std::isnan(discretized.vy.value()));
  EXPECT_FALSE(std::isinf(discretized.vx.value()));
  EXPECT_FALSE(std::isinf(discretized.vy.value()));
}

TEST(KinematicsTier2, DiscretizeNegativeRotationSymmetry) {
  frc::ChassisSpeeds posRot{3.0_mps, 0.0_mps, 3.0_rad_per_s};
  frc::ChassisSpeeds negRot{3.0_mps, 0.0_mps, -3.0_rad_per_s};

  auto discPos = frc::ChassisSpeeds::Discretize(posRot, 0.02_s);
  auto discNeg = frc::ChassisSpeeds::Discretize(negRot, 0.02_s);

  EXPECT_NEAR(discPos.vx.value(), discNeg.vx.value(), 1e-5);
  EXPECT_NEAR(discPos.vy.value(), -discNeg.vy.value(), 1e-5);
}

TEST(KinematicsTier2, DiscretizeZeroTimeDelta) {
  frc::ChassisSpeeds speeds{3.0_mps, 1.0_mps, 2.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(speeds, 0.0_s);
  EXPECT_TRUE(std::isnan(discretized.vx.value()));
  EXPECT_TRUE(std::isnan(discretized.vy.value()));
}

TEST(KinematicsTier2, DiscretizeVerySmallTimeDelta) {
  frc::ChassisSpeeds speeds{3.0_mps, 0.0_mps, 1.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(speeds, 0.001_s);
  EXPECT_NEAR(discretized.vx.value(), 3.0, 1e-4);
  EXPECT_NEAR(discretized.vy.value(), 0.0, 2e-3);
}

TEST(KinematicsTier2, DiscretizeLargeVelocityVectors) {
  frc::ChassisSpeeds highSpeed{10.0_mps, 10.0_mps, 5.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(highSpeed, 0.02_s);
  EXPECT_FALSE(std::isnan(discretized.vx.value()));
  EXPECT_FALSE(std::isnan(discretized.vy.value()));
}

// --- Feature 12 Boundary Cases ---
TEST(KinematicsTier2, SlewRateLimiterDirectionReversal) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{3.0}};
  limiter.reset(3.0_mps);

  double current = 3.0;
  for (int idx = 0; idx < 10; ++idx) {
    auto next = limiter.calculate(-3.0_mps, 0.02_s);
    EXPECT_NEAR(next.value(), current - 0.06, 1e-6);
    current = next.value();
  }
  EXPECT_NEAR(current, 3.0 - 0.6, 1e-6);
}

TEST(KinematicsTier2, SlewRateLimiterExceedingMaxAcceleration) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{1.0}};
  limiter.reset(0.0_mps);

  // Jump to 100.0 m/s
  auto step = limiter.calculate(100.0_mps, 0.02_s);
  EXPECT_NEAR(step.value(), 0.02, 1e-6);
}

TEST(KinematicsTier2, SlewRateLimiterNegativeStepLimit) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{2.0}};
  limiter.reset(0.0_mps);

  auto step = limiter.calculate(-50.0_mps, 0.02_s);
  EXPECT_NEAR(step.value(), -0.04, 1e-6);
}

TEST(KinematicsTier2, SlewRateLimiterSubThresholdStepImmediate) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{5.0}};
  limiter.reset(1.0_mps);

  // 0.01 m/s step is below 5.0 * 0.02 = 0.10 limit
  auto step = limiter.calculate(1.01_mps, 0.02_s);
  EXPECT_NEAR(step.value(), 1.01, 1e-6);
}

TEST(KinematicsTier2, SlewRateLimiterLongDurationConvergence) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{4.0}};
  limiter.reset(0.0_mps);

  // At 4.0 m/s^2, reaches 2.0 m/s in 0.5 s (25 frames of 0.02s)
  for (int frame = 0; frame < 25; ++frame) {
    limiter.calculate(2.0_mps, 0.02_s);
  }
  auto finalStep = limiter.calculate(2.0_mps, 0.02_s);
  EXPECT_NEAR(finalStep.value(), 2.0, 1e-6);
}

// --- Feature 13 Boundary Cases ---
TEST(KinematicsTier2, XPatternVectorSumEquilibrium) {
  auto states = calculateXPatternLock();

  double balX = std::cos(states[0].angle.Radians().value()) + std::cos(states[1].angle.Radians().value())
              - std::cos(states[2].angle.Radians().value()) - std::cos(states[3].angle.Radians().value());
  EXPECT_NEAR(balX, 0.0, 1e-6);
}

TEST(KinematicsTier2, XPatternNoNetTorque) {
  auto states = calculateXPatternLock();
  double netTorque = 0.0;
  for (const auto& state : states) {
    netTorque += state.speed.value();
  }
  EXPECT_DOUBLE_EQ(netTorque, 0.0);
}

TEST(KinematicsTier2, XPatternAngleNormalDirection) {
  auto states = calculateXPatternLock();
  for (const auto& state : states) {
    double deg = std::abs(state.angle.Degrees().value());
    EXPECT_NEAR(deg, 45.0, 1e-6);
  }
}

TEST(KinematicsTier2, XPatternUnitNormals) {
  auto states = calculateXPatternLock();
  for (const auto& state : states) {
    double cosVal = std::cos(state.angle.Radians().value());
    double sinVal = std::sin(state.angle.Radians().value());
    EXPECT_NEAR((cosVal * cosVal) + (sinVal * sinVal), 1.0, 1e-6);
  }
}

TEST(KinematicsTier2, XPatternDefensiveStability) {
  auto states = calculateXPatternLock();
  // With wheel speeds at 0 and orthogonal orientation, rolling resistance stops push
  for (const auto& s : states) {
    EXPECT_EQ(s.speed, 0.0_mps);
  }
}

// --- Feature 14 Boundary Cases ---
TEST(KinematicsTier2, VisionExactRotationalVelocityBoundary) {
  VisionPoseEstimate estimate{
      frc::Pose2d(1.0_m, 1.0_m, frc::Rotation2d(0.0_deg)),
      2.0_s,
      2,
      true};
  EXPECT_TRUE(evaluateVisionMeasurement(estimate, 179.9_deg_per_s, false));
  EXPECT_FALSE(evaluateVisionMeasurement(estimate, 180.1_deg_per_s, false));
}

TEST(KinematicsTier2, VisionNegativeExactRotationalVelocityBoundary) {
  VisionPoseEstimate estimate{
      frc::Pose2d(1.0_m, 1.0_m, frc::Rotation2d(0.0_deg)),
      2.0_s,
      2,
      true};
  EXPECT_TRUE(evaluateVisionMeasurement(estimate, -179.9_deg_per_s, false));
  EXPECT_FALSE(evaluateVisionMeasurement(estimate, -180.1_deg_per_s, false));
}

TEST(KinematicsTier2, VisionMissingDataFlagRejection) {
  VisionPoseEstimate estimate{
      frc::Pose2d(1.0_m, 1.0_m, frc::Rotation2d(0.0_deg)),
      2.0_s,
      2,
      false};
  EXPECT_FALSE(evaluateVisionMeasurement(estimate, 10.0_deg_per_s, true));
}

TEST(KinematicsTier2, VisionMegaTag2SingleTagPass) {
  VisionPoseEstimate estimate{
      frc::Pose2d(4.0_m, 3.0_m, frc::Rotation2d(90.0_deg)),
      3.0_s,
      1,
      true};
  EXPECT_TRUE(evaluateVisionMeasurement(estimate, 30.0_deg_per_s, true));
}

TEST(KinematicsTier2, VisionExactZeroOriginCoordinate) {
  VisionPoseEstimate estimate{
      frc::Pose2d(0.0_m, 0.0_m, frc::Rotation2d(0.0_rad)),
      3.0_s,
      4,
      true};
  EXPECT_FALSE(evaluateVisionMeasurement(estimate, 0.0_deg_per_s, true));
}

// --- Feature 15 Boundary Cases ---
TEST(KinematicsTier2, CarDriveSteeringOverRangeInput) {
  auto overSteerRight = calculateCarDrive(0.5, 0.0, 2.5, false);
  EXPECT_NEAR(overSteerRight.mSteerAngle.value(), -60.0, 1e-6);

  auto overSteerLeft = calculateCarDrive(0.5, 0.0, -5.0, false);
  EXPECT_NEAR(overSteerLeft.mSteerAngle.value(), 60.0, 1e-6);
}

TEST(KinematicsTier2, CarDriveNegativeThrottleClampedToZero) {
  auto negThrottle = calculateCarDrive(-0.5, 0.0, 0.0, false);
  EXPECT_DOUBLE_EQ(negThrottle.mDriveSpeed, 0.0);
}

TEST(KinematicsTier2, CarDriveExcessiveBrakeClampedToOne) {
  auto overBrake = calculateCarDrive(1.0, 2.0, 0.0, false);
  EXPECT_DOUBLE_EQ(overBrake.mDriveSpeed, 0.0);
}

TEST(KinematicsTier2, CarDriveFullThrottleFullBrakeZeroSpeed) {
  auto clamped = calculateCarDrive(1.0, 1.0, 0.0, false);
  EXPECT_DOUBLE_EQ(clamped.mDriveSpeed, 0.0);
}

TEST(KinematicsTier2, CarDriveReverseWithBrake) {
  auto revBrake = calculateCarDrive(0.5, 0.5, 0.0, true, 0.5);
  EXPECT_NEAR(revBrake.mDriveSpeed, -0.125, 1e-6);
}
