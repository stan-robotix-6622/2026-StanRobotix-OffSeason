#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <gtest/gtest.h>

#include <frc/MathUtil.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <networktables/NetworkTableInstance.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "stan/StanCarDrive.h"
#include "stan/StanMotor.h"
#include "stan/StanTunablePID.h"
#include "stan/StanXboxController.h"
#include "stan/SwervePresets.h"
#include "test_helpers.h"

using namespace stan;
using namespace stan::test;

// ============================================================================
// Adversarial Suite 1: Deadband Math, Extreme Inputs & Squaring Symmetry
// ============================================================================

TEST(AdversarialDeadband, FuzzingMonotonicityAcrossFullRange) {
  const double threshold = 0.08;
  double previous = -10.0;
  for (int step = -2000; step <= 2000; ++step) {
    double input = step / 1000.0; // [-2.0, 2.0]
    double output = StanXboxController::applyDeadband(input, threshold, false);
    EXPECT_GE(output, previous - 1e-12) << "Monotonicity violated at input " << input;
    previous = output;
  }
}

TEST(AdversarialDeadband, FuzzingMonotonicitySquaredResponse) {
  const double threshold = 0.08;
  double previous = -10.0;
  for (int step = -2000; step <= 2000; ++step) {
    double input = step / 1000.0;
    double output = StanXboxController::applyDeadband(input, threshold, true);
    EXPECT_GE(output, previous - 1e-12) << "Monotonicity violated in squared mode at input " << input;
    previous = output;
  }
}

TEST(AdversarialDeadband, StrictOddSymmetryLinearAndSquared) {
  const double threshold = 0.05;
  for (int step = 1; step <= 1500; ++step) {
    double input = step / 1000.0;
    double outPos = StanXboxController::applyDeadband(input, threshold, false);
    double outNeg = StanXboxController::applyDeadband(-input, threshold, false);
    EXPECT_DOUBLE_EQ(outPos, -outNeg) << "Odd symmetry violated linear at " << input;

    double sqPos = StanXboxController::applyDeadband(input, threshold, true);
    double sqNeg = StanXboxController::applyDeadband(-input, threshold, true);
    EXPECT_DOUBLE_EQ(sqPos, -sqNeg) << "Odd symmetry violated squared at " << input;
  }
}

TEST(AdversarialDeadband, InfinitesimalContinuityAroundDeadbandBoundary) {
  const double threshold = 0.05;
  const double eps = 1e-9;

  double atBoundary = StanXboxController::applyDeadband(threshold, threshold);
  EXPECT_DOUBLE_EQ(atBoundary, 0.0);

  double justAbove = StanXboxController::applyDeadband(threshold + eps, threshold);
  EXPECT_GE(justAbove, 0.0);
  EXPECT_LT(justAbove, 1e-7);

  double justBelow = StanXboxController::applyDeadband(threshold - eps, threshold);
  EXPECT_DOUBLE_EQ(justBelow, 0.0);

  double atNegBoundary = StanXboxController::applyDeadband(-threshold, threshold);
  EXPECT_DOUBLE_EQ(atNegBoundary, 0.0);

  double justBelowNeg = StanXboxController::applyDeadband(-threshold - eps, threshold);
  EXPECT_LE(justBelowNeg, 0.0);
  EXPECT_GT(justBelowNeg, -1e-7);
}

TEST(AdversarialDeadband, ZeroDeadbandThresholdPassthrough) {
  for (double val : {-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5}) {
    double out = StanXboxController::applyDeadband(val, 0.0, false);
    EXPECT_DOUBLE_EQ(out, val);
  }
}

TEST(AdversarialDeadband, ThresholdBoundaryOneDivZeroCheck) {
  // When threshold >= 1.0, (1 - threshold) causes division by zero -> NaN or Inf
  double result = StanXboxController::applyDeadband(1.0, 1.0, false);
  EXPECT_TRUE(std::isnan(result)) << "Expected NaN on threshold==1.0 with input==1.0";
}

// ============================================================================
// Adversarial Suite 2: Unit Conversions & Gear Ratio Invariants
// ============================================================================

TEST(AdversarialUnits, AngleConversionsExactMultiples) {
  units::angle::degree_t deg360{360.0_deg};
  units::angle::turn_t turns{deg360};
  EXPECT_NEAR(turns.value(), 1.0, 1e-9);

  units::angle::radian_t rads{turns};
  EXPECT_NEAR(rads.value(), 2.0 * std::numbers::pi, 1e-9);

  units::angle::degree_t backToDeg{rads};
  EXPECT_NEAR(backToDeg.value(), 360.0, 1e-9);
}

TEST(AdversarialUnits, SwervePresetsCircumferenceConsistency) {
  std::vector<SwervePreset> presets = {
      SwervePreset::kSDSMK4_L1,
      SwervePreset::kSDSMK4_L2,
      SwervePreset::kSDSMK4_L3,
      SwervePreset::kSDSMK4_L4,
      SwervePreset::kSDSMK4i_L1,
      SwervePreset::kSDSMK4i_L2,
      SwervePreset::kSDSMK4i_L3,
      SwervePreset::kSDSMK4i_L4,
      SwervePreset::kREVMAXSwerve};

  for (auto preset : presets) {
    auto constants = SwervePresets::get(preset);
    double expectedCircumference = 2.0 * std::numbers::pi * constants.kWheelRadius.value();
    EXPECT_NEAR(constants.getWheelCircumference().value(), expectedCircumference, 1e-9);
    EXPECT_NEAR(constants.getWheelDiameter().value(), 2.0 * constants.kWheelRadius.value(), 1e-9);
  }
}

TEST(AdversarialUnits, REVMAXSwerveExactGearRatios) {
  auto rev = SwervePresets::get(SwervePreset::kREVMAXSwerve);
  EXPECT_DOUBLE_EQ(rev.kDriveGearRatio, 4.71);
  EXPECT_NEAR(rev.kSteerGearRatio, 9424.0 / 203.0, 1e-6);
  EXPECT_NEAR(rev.kWheelRadius.value(), 0.0340773512, 1e-6);
}

TEST(AdversarialUnits, RPMtoTurnPerSecondConversion) {
  double rpm = 6000.0;
  units::angular_velocity::turns_per_second_t tps{rpm / 60.0};
  EXPECT_DOUBLE_EQ(tps.value(), 100.0);

  double rpmBack = tps.value() * 60.0;
  EXPECT_DOUBLE_EQ(rpmBack, 6000.0);
}

// ============================================================================
// Adversarial Suite 3: Swerve Speed Discretization & Kinematic Desaturation
// ============================================================================

TEST(AdversarialKinematics, DiscretizeHighSpeedsAndRotationalRates) {
  // At 4.5 m/s and 540 deg/s (3*pi rad/s)
  frc::ChassisSpeeds highSpeeds{4.5_mps, 0.0_mps, units::angular_velocity::radians_per_second_t{3.0 * std::numbers::pi}};
  auto discretized = frc::ChassisSpeeds::Discretize(highSpeeds, 0.02_s);

  // Vx decreases slightly because translation is rotated backwards by omega*dt/2
  EXPECT_LT(discretized.vx.value(), 4.5);
  EXPECT_GT(discretized.vx.value(), 4.3);

  // Vy becomes negative (skew correction opposes rotation)
  EXPECT_LT(discretized.vy.value(), -0.3);
  EXPECT_GT(discretized.vy.value(), -0.6);

  // Omega is preserved exactly
  EXPECT_NEAR(discretized.omega.value(), 3.0 * std::numbers::pi, 1e-9);
}

TEST(AdversarialKinematics, DiscretizeExtremeAngularRateStability) {
  // Extreme angular velocity: 100 rad/s
  frc::ChassisSpeeds extremeSpeeds{3.0_mps, 3.0_mps, 100.0_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(extremeSpeeds, 0.02_s);

  EXPECT_FALSE(std::isnan(discretized.vx.value()));
  EXPECT_FALSE(std::isnan(discretized.vy.value()));
  EXPECT_FALSE(std::isnan(discretized.omega.value()));
  EXPECT_DOUBLE_EQ(discretized.omega.value(), 100.0);
}

TEST(AdversarialKinematics, KinematicsDesaturationPreservesDirection) {
  frc::Translation2d fl{0.3_m, 0.3_m};
  frc::Translation2d fr{0.3_m, -0.3_m};
  frc::Translation2d bl{-0.3_m, 0.3_m};
  frc::Translation2d br{-0.3_m, -0.3_m};
  frc::SwerveDriveKinematics<4> kinematics{fl, fr, bl, br};

  // High chassis speed demanding wheels > 4.5 m/s
  frc::ChassisSpeeds highSpeeds{4.5_mps, 4.5_mps, 6.0_rad_per_s};
  auto moduleStates = kinematics.ToSwerveModuleStates(highSpeeds);

  // Before desaturation, some wheel speeds exceed 4.5 m/s
  bool anyExceeds = false;
  for (const auto& s : moduleStates) {
    if (s.speed > 4.5_mps) anyExceeds = true;
  }
  EXPECT_TRUE(anyExceeds);

  // After desaturation
  kinematics.DesaturateWheelSpeeds(&moduleStates, 4.5_mps);
  for (const auto& s : moduleStates) {
    EXPECT_LE(units::math::abs(s.speed).value(), 4.50001);
  }
}

// ============================================================================
// Adversarial Suite 4: Slew Rate Limiter Step Responses & Reversals
// ============================================================================

TEST(AdversarialSlewRate, StepResponseNeverExceedsRateLimit) {
  const units::meters_per_second_squared_t rateLimit{4.0};
  const units::second_t dt{0.02};
  const units::meters_per_second_t maxStep = rateLimit * dt; // 0.08 m/s

  DiscreteSlewRateLimiter limiter{rateLimit};
  limiter.reset(0.0_mps);

  units::meters_per_second_t prev = 0.0_mps;
  for (int step = 0; step < 100; ++step) {
    auto current = limiter.calculate(10.0_mps, dt);
    auto delta = current - prev;
    EXPECT_LE(delta.value(), maxStep.value() + 1e-12) << "Step " << step << " exceeded max step";
    prev = current;
  }
}

TEST(AdversarialSlewRate, SharpReversalSmoothThroughZero) {
  const units::meters_per_second_squared_t rateLimit{6.0};
  const units::second_t dt{0.02};
  const units::meters_per_second_t maxStep = rateLimit * dt; // 0.12 m/s

  DiscreteSlewRateLimiter limiter{rateLimit};
  limiter.reset(4.5_mps);

  // Command instant reverse to -4.5 m/s
  units::meters_per_second_t prev = 4.5_mps;
  bool passedZero = false;
  for (int step = 0; step < 100; ++step) {
    auto current = limiter.calculate(-4.5_mps, dt);
    auto delta = units::math::abs(current - prev);
    EXPECT_LE(delta.value(), maxStep.value() + 1e-12);
    if ((prev.value() > 0.0 && current.value() <= 0.0) ||
        (prev.value() >= 0.0 && current.value() < 0.0)) {
      passedZero = true;
    }
    prev = current;
  }
  EXPECT_TRUE(passedZero);
  EXPECT_NEAR(prev.value(), -4.5, 1e-6);
}

// ============================================================================
// Adversarial Suite 5: Vision Measurement Rejection Filter
// ============================================================================

TEST(AdversarialVision, StrictBoundaryAt180DegPerSec) {
  VisionPoseEstimate est{
      frc::Pose2d{2.0_m, 2.0_m, frc::Rotation2d{10.0_deg}},
      1.0_s,
      2,
      true};

  // Exactly 180.0 deg/s -> should be accepted (abs <= 180)
  EXPECT_TRUE(evaluateVisionMeasurement(est, 180.0_deg_per_s, false));
  EXPECT_TRUE(evaluateVisionMeasurement(est, -180.0_deg_per_s, false));

  // 180.001 deg/s -> should be rejected (abs > 180)
  EXPECT_FALSE(evaluateVisionMeasurement(est, 180.001_deg_per_s, false));
  EXPECT_FALSE(evaluateVisionMeasurement(est, -180.001_deg_per_s, false));
}

TEST(AdversarialVision, ZeroTagAndNegativeTagRejection) {
  VisionPoseEstimate est0{
      frc::Pose2d{2.0_m, 2.0_m, frc::Rotation2d{10.0_deg}},
      1.0_s,
      0,
      true};
  EXPECT_FALSE(evaluateVisionMeasurement(est0, 0.0_deg_per_s, false));

  VisionPoseEstimate estNeg{
      frc::Pose2d{2.0_m, 2.0_m, frc::Rotation2d{10.0_deg}},
      1.0_s,
      -1,
      true};
  // In MegaTag1 mode, tagCount < 2 is rejected
  EXPECT_FALSE(evaluateVisionMeasurement(estNeg, 0.0_deg_per_s, false));
}

TEST(AdversarialVision, OriginPoseRejection) {
  VisionPoseEstimate estOrigin{
      frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}},
      1.0_s,
      2,
      true};
  EXPECT_FALSE(evaluateVisionMeasurement(estOrigin, 0.0_deg_per_s, false));

  // Non-origin pose should pass
  VisionPoseEstimate estNearOrigin{
      frc::Pose2d{0.01_m, 0.0_m, frc::Rotation2d{0.0_deg}},
      1.0_s,
      2,
      true};
  EXPECT_TRUE(evaluateVisionMeasurement(estNearOrigin, 0.0_deg_per_s, false));
}

// ============================================================================
// Adversarial Suite 6: CarDrive / Ackermann Throttle & Brake Blend
// ============================================================================

TEST(AdversarialCarDrive, FullGridThrottleBrakeInteraction) {
  for (int t = 0; t <= 10; ++t) {
    for (int b = 0; b <= 10; ++b) {
      double throttle = t / 10.0;
      double brake = b / 10.0;

      auto outForward = calculateCarDrive(throttle, brake, 0.0, false, 0.5, 60.0_deg);
      auto outReverse = calculateCarDrive(throttle, brake, 0.0, true, 0.5, 60.0_deg);

      // Speed must never be negative in forward mode
      EXPECT_GE(outForward.mDriveSpeed, 0.0);
      EXPECT_LE(outReverse.mDriveSpeed, 0.0);

      // Full brake always forces speed to zero
      if (brake >= 1.0) {
        EXPECT_DOUBLE_EQ(outForward.mDriveSpeed, 0.0);
        EXPECT_DOUBLE_EQ(outReverse.mDriveSpeed, 0.0);
      }

      // Reverse is strictly symmetric
      EXPECT_DOUBLE_EQ(outForward.mDriveSpeed, -outReverse.mDriveSpeed);
    }
  }
}

TEST(AdversarialCarDrive, SteerAngleStrictBoundaryClamping) {
  auto outCenter = calculateCarDrive(1.0, 0.0, 0.0, false, 0.5, 60.0_deg);
  EXPECT_DOUBLE_EQ(outCenter.mSteerAngle.value(), 0.0);

  auto outFullRight = calculateCarDrive(1.0, 0.0, 1.0, false, 0.5, 60.0_deg);
  EXPECT_DOUBLE_EQ(outFullRight.mSteerAngle.value(), -60.0);

  auto outFullLeft = calculateCarDrive(1.0, 0.0, -1.0, false, 0.5, 60.0_deg);
  EXPECT_DOUBLE_EQ(outFullLeft.mSteerAngle.value(), 60.0);

  // Over-range steer inputs are strictly clamped to max steer angle
  auto outOverRight = calculateCarDrive(1.0, 0.0, 2.5, false, 0.5, 60.0_deg);
  EXPECT_DOUBLE_EQ(outOverRight.mSteerAngle.value(), -60.0);

  auto outOverLeft = calculateCarDrive(1.0, 0.0, -3.0, false, 0.5, 60.0_deg);
  EXPECT_DOUBLE_EQ(outOverLeft.mSteerAngle.value(), 60.0);
}

// ============================================================================
// Adversarial Suite 7: StanTunablePID & NT4 Dirty Flag Latching
// ============================================================================

TEST(AdversarialTunablePID, LiveInstanceNetworkTablesPublishAndDirtyFlagLatch) {
  PIDGains defaultGains{0.2, 0.01, 0.05, 0.1, 0.15, 0.02};
  StanTunablePID tunablePID{"TestAdversarialPID", defaultGains};

  // On construction, table exists and gains match defaults
  auto table = tunablePID.getTable();
  ASSERT_NE(table, nullptr);
  EXPECT_DOUBLE_EQ(table->GetNumber("kP", 0.0), 0.2);
  EXPECT_DOUBLE_EQ(table->GetNumber("kI", 0.0), 0.01);
  EXPECT_DOUBLE_EQ(table->GetNumber("kD", 0.0), 0.05);
  EXPECT_DOUBLE_EQ(table->GetNumber("kS", 0.0), 0.1);
  EXPECT_DOUBLE_EQ(table->GetNumber("kV", 0.0), 0.15);
  EXPECT_DOUBLE_EQ(table->GetNumber("kG", 0.0), 0.02);

  // Before any change, hasChanged() must be false
  EXPECT_FALSE(tunablePID.hasChanged());

  // Simulate remote dashboard change to kP
  table->PutNumber("kP", 0.35);

  // hasChanged() should detect change and latch
  EXPECT_TRUE(tunablePID.hasChanged());
  EXPECT_DOUBLE_EQ(tunablePID.getGains().kP, 0.35);

  // Subsequent call without changes must return false (latch cleared)
  EXPECT_FALSE(tunablePID.hasChanged());

  // Multiple changes at once
  table->PutNumber("kD", 0.12);
  table->PutNumber("kV", 0.25);
  EXPECT_TRUE(tunablePID.hasChanged());
  EXPECT_DOUBLE_EQ(tunablePID.getGains().kD, 0.12);
  EXPECT_DOUBLE_EQ(tunablePID.getGains().kV, 0.25);
  EXPECT_FALSE(tunablePID.hasChanged());
}

TEST(AdversarialTunablePID, PresetBaselineEquivalence) {
  auto flywheel = StanTunablePID::getPreset(TunablePreset::kFlywheel);
  EXPECT_DOUBLE_EQ(flywheel.kP, 0.1);
  EXPECT_DOUBLE_EQ(flywheel.kS, 0.05);
  EXPECT_DOUBLE_EQ(flywheel.kV, 0.12);

  auto pivot = StanTunablePID::getPreset(TunablePreset::kPivot);
  EXPECT_DOUBLE_EQ(pivot.kP, 40.0);
  EXPECT_DOUBLE_EQ(pivot.kD, 0.5);
  EXPECT_DOUBLE_EQ(pivot.kG, 0.3);

  auto roller = StanTunablePID::getPreset(TunablePreset::kRoller);
  EXPECT_DOUBLE_EQ(roller.kP, 0.05);

  auto drive = StanTunablePID::getPreset(TunablePreset::kSwerveDrive);
  EXPECT_DOUBLE_EQ(drive.kP, 0.1);

  auto steer = StanTunablePID::getPreset(TunablePreset::kSwerveSteer);
  EXPECT_DOUBLE_EQ(steer.kP, 40.0);
  EXPECT_DOUBLE_EQ(steer.kD, 0.5);
}
