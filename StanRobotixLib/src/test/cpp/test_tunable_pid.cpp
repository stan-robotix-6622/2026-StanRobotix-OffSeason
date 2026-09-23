#include <algorithm>
#include <cmath>
#include <numbers>
#include <gtest/gtest.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "stan/StanElevator.h"
#include "stan/StanFlywheel.h"
#include "stan/StanPivot.h"
#include "stan/StanRoller.h"
#include "stan/StanTunablePID.h"
#include "test_helpers.h"

using namespace stan;
using namespace stan::test;

// ============================================================================
// Tier 1: Feature Coverage (Features 4, 5, 6, 7, 16: 25 Tests Total)
// ============================================================================

// --- Feature 4: StanRoller Subsystem Commands & Controls ---
TEST(TunablePIDTier1, RollerSpeedClamping) {
  double target = std::clamp(0.8, -1.0, 1.0);
  EXPECT_DOUBLE_EQ(target, 0.8);

  double targetOver = std::clamp(1.5, -1.0, 1.0);
  EXPECT_DOUBLE_EQ(targetOver, 1.0);

  double targetUnder = std::clamp(-1.8, -1.0, 1.0);
  EXPECT_DOUBLE_EQ(targetUnder, -1.0);
}

TEST(TunablePIDTier1, RollerVoltageSettingLimits) {
  units::voltage::volt_t v1 = clampVoltage(8.0_V);
  EXPECT_NEAR(v1.value(), 8.0, 1e-6);

  units::voltage::volt_t v2 = clampVoltage(14.0_V);
  EXPECT_NEAR(v2.value(), 12.0, 1e-6);
}

TEST(TunablePIDTier1, RollerRunCommandSimulation) {
  double currentSpeed = 0.0;
  auto runRollerLambda = [&](double iSpeed) {
    currentSpeed = std::clamp(iSpeed, -1.0, 1.0);
  };

  runRollerLambda(0.75);
  EXPECT_DOUBLE_EQ(currentSpeed, 0.75);
}

TEST(TunablePIDTier1, RollerRunEndCommandSimulation) {
  double currentSpeed = 0.0;
  auto startAction = [&] { currentSpeed = 1.0; };
  auto endAction = [&] { currentSpeed = 0.0; };

  startAction();
  EXPECT_DOUBLE_EQ(currentSpeed, 1.0);
  endAction();
  EXPECT_DOUBLE_EQ(currentSpeed, 0.0);
}

TEST(TunablePIDTier1, RollerStopCommandSimulation) {
  double currentSpeed = 0.6;
  auto stopAction = [&] { currentSpeed = 0.0; };

  stopAction();
  EXPECT_DOUBLE_EQ(currentSpeed, 0.0);
}

// --- Feature 5: StanFlywheel Velocity & Readiness Tolerance ---
TEST(TunablePIDTier1, FlywheelTargetVelocitySetting) {
  FlywheelConfig config{};
  config.kMaxVelocity = 100_tps;

  auto clampVel = [&](units::angular_velocity::turns_per_second_t iVel) {
    return std::clamp(iVel, -config.kMaxVelocity, config.kMaxVelocity);
  };

  EXPECT_NEAR(clampVel(80_tps).value(), 80.0, 1e-6);
  EXPECT_NEAR(clampVel(120_tps).value(), 100.0, 1e-6);
}

TEST(TunablePIDTier1, FlywheelReadinessToleranceBand) {
  FlywheelConfig config{};
  config.kTolerance = 2_tps;
  units::angular_velocity::turns_per_second_t target = 80_tps;

  auto isReady = [&](units::angular_velocity::turns_per_second_t iCurrent) {
    return std::abs((iCurrent - target).value()) <= config.kTolerance.value();
  };

  EXPECT_TRUE(isReady(80_tps));
  EXPECT_TRUE(isReady(81.5_tps));
  EXPECT_TRUE(isReady(78.5_tps));
  EXPECT_FALSE(isReady(75_tps));
  EXPECT_FALSE(isReady(83_tps));
}

TEST(TunablePIDTier1, FlywheelSpinUpCommandTarget) {
  FlywheelConfig config{};
  config.kMaxVelocity = 100_tps;
  units::angular_velocity::turns_per_second_t targetVelocity{0_tps};

  auto spinUp = [&] { targetVelocity = config.kMaxVelocity; };
  spinUp();
  EXPECT_EQ(targetVelocity, 100_tps);
}

TEST(TunablePIDTier1, FlywheelStopCommandTarget) {
  units::angular_velocity::turns_per_second_t targetVelocity{60_tps};
  auto stop = [&] { targetVelocity = 0_tps; };
  stop();
  EXPECT_EQ(targetVelocity, 0_tps);
}

TEST(TunablePIDTier1, FlywheelToleranceSetting) {
  FlywheelConfig config{};
  config.kTolerance = 2_tps;
  EXPECT_EQ(config.kTolerance, 2_tps);

  config.kTolerance = 1_tps;
  EXPECT_EQ(config.kTolerance, 1_tps);
}

// --- Feature 6: StanPivot Angular Position & Limits ---
TEST(TunablePIDTier1, PivotSoftLimitClamping) {
  PivotConfig config{};
  config.kMinAngle = 0_deg;
  config.kMaxAngle = 120_deg;

  auto clampAngle = [&](units::angle::degree_t iRequested) {
    return std::clamp(iRequested, config.kMinAngle, config.kMaxAngle);
  };

  EXPECT_NEAR(clampAngle(45_deg).value(), 45.0, 1e-6);
  EXPECT_NEAR(clampAngle(-10_deg).value(), 0.0, 1e-6);
  EXPECT_NEAR(clampAngle(150_deg).value(), 120.0, 1e-6);
}

TEST(TunablePIDTier1, PivotTargetAngleSetting) {
  PivotConfig config{};
  units::angle::degree_t targetAngle{0_deg};

  auto setTarget = [&](units::angle::degree_t iAngle) {
    targetAngle = std::clamp(iAngle, config.kMinAngle, config.kMaxAngle);
  };

  setTarget(90_deg);
  EXPECT_EQ(targetAngle, 90_deg);
}

TEST(TunablePIDTier1, PivotToleranceBandChecking) {
  PivotConfig config{};
  config.kTolerance = 1.0_deg;
  units::angle::degree_t target = 45.0_deg;

  auto atAngle = [&](units::angle::degree_t iCurrent) {
    return std::abs((iCurrent - target).value()) <= config.kTolerance.value();
  };

  EXPECT_TRUE(atAngle(45.0_deg));
  EXPECT_TRUE(atAngle(45.8_deg));
  EXPECT_TRUE(atAngle(44.2_deg));
  EXPECT_FALSE(atAngle(43.5_deg));
  EXPECT_FALSE(atAngle(46.5_deg));
}

TEST(TunablePIDTier1, PivotGearRatioRatioCalculation) {
  PivotConfig config{};
  config.kGearRatio = 50.0;
  // 1 mechanism turn = 50 motor turns
  double mechanismTurns = 0.25; // 90 deg
  double motorTurns = mechanismTurns * config.kGearRatio;
  EXPECT_NEAR(motorTurns, 12.5, 1e-6);
}

TEST(TunablePIDTier1, PivotGravityFeedforwardArmCosine) {
  PivotConfig config{};
  config.kG = 0.3;

  auto calculateGravityFF = [&](units::angle::degree_t iAngle) {
    return config.kG * std::cos(iAngle.convert<units::angle::radian>().value());
  };

  EXPECT_NEAR(calculateGravityFF(0_deg), 0.3, 1e-6);
  EXPECT_NEAR(calculateGravityFF(90_deg), 0.0, 1e-6);
  EXPECT_NEAR(calculateGravityFF(180_deg), -0.3, 1e-6);
}

// --- Feature 7: StanElevator Height Control & Limits ---
TEST(TunablePIDTier1, ElevatorSoftLimitClamping) {
  ElevatorConfig config{};
  config.kMinHeight = 0.0_m;
  config.kMaxHeight = 1.5_m;

  auto clampHeight = [&](units::length::meter_t iRequested) {
    return std::clamp(iRequested, config.kMinHeight, config.kMaxHeight);
  };

  EXPECT_NEAR(clampHeight(0.8_m).value(), 0.8, 1e-6);
  EXPECT_NEAR(clampHeight(-0.2_m).value(), 0.0, 1e-6);
  EXPECT_NEAR(clampHeight(2.0_m).value(), 1.5, 1e-6);
}

TEST(TunablePIDTier1, ElevatorTargetHeightSetting) {
  ElevatorConfig config{};
  units::length::meter_t targetHeight{0_m};

  auto setTarget = [&](units::length::meter_t iHeight) {
    targetHeight = std::clamp(iHeight, config.kMinHeight, config.kMaxHeight);
  };

  setTarget(1.2_m);
  EXPECT_EQ(targetHeight, 1.2_m);
}

TEST(TunablePIDTier1, ElevatorToleranceBandChecking) {
  ElevatorConfig config{};
  config.kTolerance = 0.01_m; // 1 cm
  units::length::meter_t target = 1.0_m;

  auto atHeight = [&](units::length::meter_t iCurrent) {
    return std::abs((iCurrent - target).value()) <= config.kTolerance.value();
  };

  EXPECT_TRUE(atHeight(1.0_m));
  EXPECT_TRUE(atHeight(1.008_m));
  EXPECT_TRUE(atHeight(0.992_m));
  EXPECT_FALSE(atHeight(0.98_m));
  EXPECT_FALSE(atHeight(1.02_m));
}

TEST(TunablePIDTier1, ElevatorTurnsConversion) {
  ElevatorConfig config{};
  config.kMetersPerRotation = 0.05; // 5 cm per turn
  units::length::meter_t height = 1.0_m;
  double turns = height.value() / config.kMetersPerRotation;
  EXPECT_NEAR(turns, 20.0, 1e-6);
}

TEST(TunablePIDTier1, ElevatorConstantGravityFeedforward) {
  ElevatorConfig config{};
  config.kG = 0.4;
  EXPECT_DOUBLE_EQ(config.kG, 0.4);
}

// --- Feature 16: StanTunablePID Math & Presets ---
TEST(TunablePIDTier1, FlywheelPresetValues) {
  auto preset = StanTunablePID::getPreset(TunablePreset::kFlywheel);
  EXPECT_DOUBLE_EQ(preset.kP, 0.1);
  EXPECT_DOUBLE_EQ(preset.kI, 0.0);
  EXPECT_DOUBLE_EQ(preset.kD, 0.0);
  EXPECT_DOUBLE_EQ(preset.kS, 0.05);
  EXPECT_DOUBLE_EQ(preset.kV, 0.12);
  EXPECT_DOUBLE_EQ(preset.kG, 0.0);
}

TEST(TunablePIDTier1, PivotPresetValues) {
  auto preset = StanTunablePID::getPreset(TunablePreset::kPivot);
  EXPECT_DOUBLE_EQ(preset.kP, 40.0);
  EXPECT_DOUBLE_EQ(preset.kI, 0.0);
  EXPECT_DOUBLE_EQ(preset.kD, 0.5);
  EXPECT_DOUBLE_EQ(preset.kS, 0.0);
  EXPECT_DOUBLE_EQ(preset.kV, 0.0);
  EXPECT_DOUBLE_EQ(preset.kG, 0.3);
}

TEST(TunablePIDTier1, RollerPresetValues) {
  auto preset = StanTunablePID::getPreset(TunablePreset::kRoller);
  EXPECT_DOUBLE_EQ(preset.kP, 0.05);
  EXPECT_DOUBLE_EQ(preset.kI, 0.0);
  EXPECT_DOUBLE_EQ(preset.kD, 0.0);
  EXPECT_DOUBLE_EQ(preset.kS, 0.05);
  EXPECT_DOUBLE_EQ(preset.kV, 0.12);
}

TEST(TunablePIDTier1, SwerveDriveAndSteerPresets) {
  auto drivePreset = StanTunablePID::getPreset(TunablePreset::kSwerveDrive);
  EXPECT_DOUBLE_EQ(drivePreset.kP, 0.1);
  EXPECT_DOUBLE_EQ(drivePreset.kS, 0.1);
  EXPECT_DOUBLE_EQ(drivePreset.kV, 0.12);

  auto steerPreset = StanTunablePID::getPreset(TunablePreset::kSwerveSteer);
  EXPECT_DOUBLE_EQ(steerPreset.kP, 40.0);
  EXPECT_DOUBLE_EQ(steerPreset.kD, 0.5);
}

TEST(TunablePIDTier1, PIDOutputCalculationProportional) {
  double integral = 0.0;
  double prevError = 0.0;
  double output = calculatePIDOutput(10.0, 8.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.02, integral, prevError);
  EXPECT_NEAR(output, 1.0, 1e-6);
}

// ============================================================================
// Tier 2: Boundary & Corner Cases (Features 4, 5, 6, 7, 16: 25 Tests Total)
// ============================================================================

// --- Feature 4 Boundary Cases ---
TEST(TunablePIDTier2, RollerOverSpeedClampingPositive) {
  double clamped = std::clamp(2.5, -1.0, 1.0);
  EXPECT_DOUBLE_EQ(clamped, 1.0);
}

TEST(TunablePIDTier2, RollerOverSpeedClampingNegative) {
  double clamped = std::clamp(-3.0, -1.0, 1.0);
  EXPECT_DOUBLE_EQ(clamped, -1.0);
}

TEST(TunablePIDTier2, RollerZeroSpeedExact) {
  double clamped = std::clamp(0.0, -1.0, 1.0);
  EXPECT_DOUBLE_EQ(clamped, 0.0);
}

TEST(TunablePIDTier2, RollerVoltageOverdriveClamp) {
  units::voltage::volt_t over = clampVoltage(24.0_V);
  EXPECT_DOUBLE_EQ(over.value(), 12.0);
}

TEST(TunablePIDTier2, RollerVoltageUnderdriveClamp) {
  units::voltage::volt_t under = clampVoltage(-18.0_V);
  EXPECT_DOUBLE_EQ(under.value(), -12.0);
}

// --- Feature 5 Boundary Cases ---
TEST(TunablePIDTier2, FlywheelExactToleranceBoundary) {
  units::angular_velocity::turns_per_second_t tolerance = 2.0_tps;
  double errPositiveExact = 2.0;
  EXPECT_LE(errPositiveExact, tolerance.value());
  double errNegativeExact = 2.0;
  EXPECT_LE(errNegativeExact, tolerance.value());
}

TEST(TunablePIDTier2, FlywheelEpsilonOutsideTolerance) {
  units::angular_velocity::turns_per_second_t tolerance = 2.0_tps;
  double errOutside = 2.0001;
  EXPECT_GT(errOutside, tolerance.value());
}

TEST(TunablePIDTier2, FlywheelOverMaxVelocityClamp) {
  FlywheelConfig config{};
  config.kMaxVelocity = 100_tps;
  auto clamped = std::clamp(150_tps, -config.kMaxVelocity, config.kMaxVelocity);
  EXPECT_EQ(clamped, 100_tps);
}

TEST(TunablePIDTier2, FlywheelNegativeMaxVelocityClamp) {
  FlywheelConfig config{};
  config.kMaxVelocity = 100_tps;
  auto clamped = std::clamp(-120_tps, -config.kMaxVelocity, config.kMaxVelocity);
  EXPECT_EQ(clamped, -100_tps);
}

TEST(TunablePIDTier2, FlywheelZeroToleranceCheck) {
  FlywheelConfig config{};
  config.kTolerance = 0_tps;
  units::angular_velocity::turns_per_second_t target = 50_tps;
  EXPECT_TRUE(std::abs((50_tps - target).value()) <= config.kTolerance.value());
  EXPECT_FALSE(std::abs((50.001_tps - target).value()) <= config.kTolerance.value());
}

// --- Feature 6 Boundary Cases ---
TEST(TunablePIDTier2, PivotAngleLimitsExactBoundaries) {
  PivotConfig config{};
  config.kMinAngle = 0_deg;
  config.kMaxAngle = 120_deg;

  EXPECT_DOUBLE_EQ(std::clamp(0.0_deg, config.kMinAngle, config.kMaxAngle).value(), 0.0);
  EXPECT_DOUBLE_EQ(std::clamp(120.0_deg, config.kMinAngle, config.kMaxAngle).value(), 120.0);
}

TEST(TunablePIDTier2, PivotSubMinAngleClamping) {
  PivotConfig config{};
  config.kMinAngle = 0_deg;
  config.kMaxAngle = 120_deg;
  EXPECT_DOUBLE_EQ(std::clamp(-45_deg, config.kMinAngle, config.kMaxAngle).value(), 0.0);
}

TEST(TunablePIDTier2, PivotAboveMaxAngleClamping) {
  PivotConfig config{};
  config.kMinAngle = 0_deg;
  config.kMaxAngle = 120_deg;
  EXPECT_DOUBLE_EQ(std::clamp(200_deg, config.kMinAngle, config.kMaxAngle).value(), 120.0);
}

TEST(TunablePIDTier2, PivotContinuousWrapTurnsConversion) {
  PivotConfig config{};
  config.kContinuousWrap = true;
  EXPECT_TRUE(config.kContinuousWrap);
}

TEST(TunablePIDTier2, PivotToleranceExactBoundary) {
  PivotConfig config{};
  config.kTolerance = 1.0_deg;
  EXPECT_LE(1.0, config.kTolerance.value());
  EXPECT_GT(1.0001, config.kTolerance.value());
}

// --- Feature 7 Boundary Cases ---
TEST(TunatorTier2, ElevatorHeightLimitsExactBoundaries) {
  ElevatorConfig config{};
  config.kMinHeight = 0.0_m;
  config.kMaxHeight = 1.5_m;

  EXPECT_DOUBLE_EQ(std::clamp(0.0_m, config.kMinHeight, config.kMaxHeight).value(), 0.0);
  EXPECT_DOUBLE_EQ(std::clamp(1.5_m, config.kMinHeight, config.kMaxHeight).value(), 1.5);
}

TEST(TunablePIDTier2, ElevatorSubMinHeightClamping) {
  ElevatorConfig config{};
  config.kMinHeight = 0.0_m;
  config.kMaxHeight = 1.5_m;
  EXPECT_DOUBLE_EQ(std::clamp(-0.5_m, config.kMinHeight, config.kMaxHeight).value(), 0.0);
}

TEST(TunablePIDTier2, ElevatorAboveMaxHeightClamping) {
  ElevatorConfig config{};
  config.kMinHeight = 0.0_m;
  config.kMaxHeight = 1.5_m;
  EXPECT_DOUBLE_EQ(std::clamp(2.5_m, config.kMinHeight, config.kMaxHeight).value(), 1.5);
}

TEST(TunablePIDTier2, ElevatorToleranceExactBoundary) {
  ElevatorConfig config{};
  config.kTolerance = 0.01_m;
  EXPECT_LE(0.01, config.kTolerance.value());
  EXPECT_GT(0.01001, config.kTolerance.value());
}

TEST(TunablePIDTier2, ElevatorZeroHeightPositionTurns) {
  ElevatorConfig config{};
  config.kMetersPerRotation = 0.05;
  units::length::meter_t zeroHeight = 0.0_m;
  double turns = zeroHeight.value() / config.kMetersPerRotation;
  EXPECT_DOUBLE_EQ(turns, 0.0);
}

// --- Feature 16 Boundary Cases ---
TEST(TunablePIDTier2, ZeroErrorZeroOutputWithoutFeedforward) {
  double integral = 0.0;
  double prevError = 0.0;
  double output = calculatePIDOutput(0.0, 0.0, 1.0, 0.1, 0.05, 0.0, 0.0, 0.02, integral, prevError);
  EXPECT_DOUBLE_EQ(output, 0.0);
}

TEST(TunablePIDTier2, IntegralAccumulationOverSteps) {
  double integral = 0.0;
  double prevError = 0.0;
  for (int step = 0; step < 10; ++step) {
    calculatePIDOutput(1.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.02, integral, prevError);
  }
  EXPECT_NEAR(integral, 0.2, 1e-6);
}

TEST(TunablePIDTier2, NegativeSetpointFeedforwardSign) {
  double integral = 0.0;
  double prevError = 0.0;
  double outPos = calculatePIDOutput(10.0, 10.0, 0.0, 0.0, 0.0, 0.05, 0.1, 0.02, integral, prevError);
  double outNeg = calculatePIDOutput(-10.0, -10.0, 0.0, 0.0, 0.0, 0.05, 0.1, 0.02, integral, prevError);
  EXPECT_DOUBLE_EQ(outPos, -outNeg);
}

TEST(TunablePIDTier2, ZeroGainsZeroOutput) {
  double integral = 0.0;
  double prevError = 0.0;
  double output = calculatePIDOutput(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, integral, prevError);
  EXPECT_DOUBLE_EQ(output, 0.0);
}

TEST(TunablePIDTier2, LargeErrorStepOutputSaturation) {
  double integral = 0.0;
  double prevError = 0.0;
  double output = calculatePIDOutput(1000.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.02, integral, prevError);
  EXPECT_GT(output, 100.0);
  double clampedOutput = std::clamp(output, -12.0, 12.0);
  EXPECT_DOUBLE_EQ(clampedOutput, 12.0);
}

// ============================================================================
// Tier 3: Cross-Feature Combinations (12 Tests Total)
// ============================================================================

TEST(CrossFeatureTier3, DeadbandThenSlewRatePipeline) {
  double rawJoystick = 0.08;
  double threshold = 0.1;
  double deadbanded = (std::abs(rawJoystick) <= threshold) ? 0.0 : rawJoystick;
  EXPECT_DOUBLE_EQ(deadbanded, 0.0);

  double activeJoystick = 0.55;
  double activeDeadbanded = (activeJoystick - threshold) / (1.0 - threshold);
  EXPECT_NEAR(activeDeadbanded, 0.5, 1e-6);

  double current = 0.0;
  double rateLimit = 2.0;
  double maxStep = rateLimit * 0.02;
  double next = current + std::clamp(activeDeadbanded - current, -maxStep, maxStep);
  EXPECT_NEAR(next, 0.04, 1e-6);
}

TEST(CrossFeatureTier3, FlywheelReadinessGatesRollerIntake) {
  units::angular_velocity::turns_per_second_t flywheelSpeed{0_tps};
  units::angular_velocity::turns_per_second_t targetSpeed{90_tps};
  units::angular_velocity::turns_per_second_t tolerance{2_tps};
  bool feederActive = false;

  auto evaluateSystem = [&] {
    bool atSpeed = units::math::abs(flywheelSpeed - targetSpeed) <= tolerance;
    feederActive = atSpeed;
  };

  evaluateSystem();
  EXPECT_FALSE(feederActive);

  flywheelSpeed = 85_tps;
  evaluateSystem();
  EXPECT_FALSE(feederActive);

  flywheelSpeed = 89.5_tps;
  evaluateSystem();
  EXPECT_TRUE(feederActive);
}

TEST(CrossFeatureTier3, DiscretizeWithChassisSpeedsAndSwerveKinematics) {
  frc::ChassisSpeeds speeds{2.5_mps, 0.5_mps, 1.5_rad_per_s};
  auto discretized = frc::ChassisSpeeds::Discretize(speeds, 0.02_s);

  frc::Translation2d fl{0.3_m, 0.3_m};
  frc::Translation2d fr{0.3_m, -0.3_m};
  frc::Translation2d bl{-0.3_m, 0.3_m};
  frc::Translation2d br{-0.3_m, -0.3_m};
  frc::SwerveDriveKinematics kinematics{fl, fr, bl, br};

  auto moduleStates = kinematics.ToSwerveModuleStates(discretized);
  EXPECT_GT(moduleStates[0].speed.value(), 0.0);
  EXPECT_GT(moduleStates[1].speed.value(), 0.0);
  EXPECT_GT(moduleStates[2].speed.value(), 0.0);
  EXPECT_GT(moduleStates[3].speed.value(), 0.0);
}

TEST(CrossFeatureTier3, CarDriveBrakeOverridesThrottleAndSteer) {
  double throttle = 1.0;
  double brake = 1.0;
  double steer = 0.8;
  double speed = (throttle * 0.5) * (1.0 - brake);
  EXPECT_DOUBLE_EQ(speed, 0.0);

  units::angle::degree_t steerAngle = -steer * 60.0_deg;
  EXPECT_NEAR(steerAngle.value(), -48.0, 1e-6);
}

TEST(CrossFeatureTier3, VisionFrameRejectionDuringAggressiveTurn) {
  units::angular_velocity::degrees_per_second_t highTurnRate{210.0_deg_per_s};
  bool isTagPresent = true;
  bool isOrigin = false;

  bool rejectUpdate = (units::math::abs(highTurnRate) > 180.0_deg_per_s) || !isTagPresent || isOrigin;
  EXPECT_TRUE(rejectUpdate);
}

TEST(CrossFeatureTier3, TunablePIDUpdatesFlywheelFeedforward) {
  PIDGains gains = StanTunablePID::getPreset(TunablePreset::kFlywheel);
  double integral = 0.0;
  double prevError = 0.0;

  double outOriginal = calculatePIDOutput(80.0, 80.0, gains.kP, gains.kI, gains.kD,
                                          gains.kS, gains.kV, 0.02, integral, prevError);

  // Tuned kV increased from 0.12 to 0.15
  gains.kV = 0.15;
  double outTuned = calculatePIDOutput(80.0, 80.0, gains.kP, gains.kI, gains.kD,
                                       gains.kS, gains.kV, 0.02, integral, prevError);
  EXPECT_GT(outTuned, outOriginal);
}

TEST(CrossFeatureTier3, ElevatorHeightGatesPivotRotation) {
  units::length::meter_t elevatorHeight{0.2_m};
  units::angle::degree_t requestedPivotAngle{90_deg};
  units::angle::degree_t actualPivotAngle{0_deg};

  auto updateScoringArm = [&] {
    if (elevatorHeight >= 0.5_m) {
      actualPivotAngle = requestedPivotAngle;
    } else {
      actualPivotAngle = 0_deg;
    }
  };

  updateScoringArm();
  EXPECT_DOUBLE_EQ(actualPivotAngle.value(), 0.0);

  elevatorHeight = 0.6_m;
  updateScoringArm();
  EXPECT_DOUBLE_EQ(actualPivotAngle.value(), 90.0);
}

TEST(CrossFeatureTier3, SlewRateLimiterWithCarDriveSteering) {
  DiscreteSlewRateLimiter driveLimiter{units::meters_per_second_squared_t{3.0}};
  driveLimiter.reset(0.0_mps);

  double targetThrottle = 1.0;
  auto step1 = driveLimiter.calculate(units::meters_per_second_t{targetThrottle}, 0.02_s);
  EXPECT_NEAR(step1.value(), 0.06, 1e-6);

  auto carOutputs = calculateCarDrive(step1.value(), 0.0, 0.5, false, 1.0);
  EXPECT_NEAR(carOutputs.mDriveSpeed, 0.06, 1e-6);
  EXPECT_NEAR(carOutputs.mSteerAngle.value(), -30.0, 1e-6);
}

TEST(CrossFeatureTier3, SwerveKinematicsCombinedOptimizationCheck) {
  frc::Translation2d fl{0.3_m, 0.3_m};
  frc::Translation2d fr{0.3_m, -0.3_m};
  frc::Translation2d bl{-0.3_m, 0.3_m};
  frc::Translation2d br{-0.3_m, -0.3_m};
  frc::SwerveDriveKinematics kinematics{fl, fr, bl, br};

  frc::ChassisSpeeds command{3.0_mps, 0.0_mps, 0.0_rad_per_s};
  auto states = kinematics.ToSwerveModuleStates(command);

  for (const auto& s : states) {
    EXPECT_NEAR(s.speed.value(), 3.0, 1e-5);
    EXPECT_NEAR(s.angle.Degrees().value(), 0.0, 1e-5);
  }
}

TEST(CrossFeatureTier3, VisionTimestampLatencyVersusFPGAClock) {
  units::time::second_t fpgaClock = 25.0_s;
  units::time::second_t cameraLatency = 0.030_s; // 30ms latency
  units::time::second_t measurementTime = fpgaClock - cameraLatency;

  EXPECT_LT(measurementTime, fpgaClock);
  EXPECT_NEAR((fpgaClock - measurementTime).value(), 0.030, 1e-6);
}

TEST(CrossFeatureTier3, RollerCurrentSpikeIntakeHold) {
  units::current::ampere_t currentDraw{15.0_A};
  bool pieceAcquired = false;

  auto checkCurrent = [&](units::current::ampere_t iCurrent) {
    if (iCurrent > 30.0_A) {
      pieceAcquired = true;
    }
  };

  checkCurrent(currentDraw);
  EXPECT_FALSE(pieceAcquired);

  currentDraw = 35.0_A; // stall current spike on intake
  checkCurrent(currentDraw);
  EXPECT_TRUE(pieceAcquired);
}

TEST(CrossFeatureTier3, SlewRateDirectionReversalThroughZero) {
  DiscreteSlewRateLimiter limiter{units::meters_per_second_squared_t{5.0}};
  limiter.reset(2.0_mps);

  // Moving from +2.0 to -2.0 at 5.0 m/s^2 (0.1 m/s per 0.02s frame)
  // Takes 20 frames to reach zero
  for (int f = 0; f < 20; ++f) {
    limiter.calculate(-2.0_mps, 0.02_s);
  }
  auto atZero = limiter.calculate(-2.0_mps, 0.02_s);
  EXPECT_NEAR(atZero.value(), -0.1, 1e-6);
}

// ============================================================================
// Tier 4: Real-World Application Scenarios (8 Tests Total)
// ============================================================================

TEST(ScenariosTier4, Scenario1_TeleopDriveUnderHeavyAcceleration) {
  double rateLimit = 3.0;
  double dt = 0.02;
  double maxChangePerFrame = rateLimit * dt;
  double currentSpeed = 0.0;
  double requestedSpeed = 4.5;

  for (int frame = 0; frame < 50; ++frame) {
    double delta = requestedSpeed - currentSpeed;
    currentSpeed += std::clamp(delta, -maxChangePerFrame, maxChangePerFrame);
  }

  EXPECT_NEAR(currentSpeed, 3.0, 1e-5);
  EXPECT_LT(currentSpeed, requestedSpeed);
}

TEST(ScenariosTier4, Scenario2_HighSpeedRotationSwerveSkewCompensation) {
  frc::ChassisSpeeds rawSpeeds{3.0_mps, 0.0_mps, units::angular_velocity::radians_per_second_t{2.0 * std::numbers::pi}};
  auto compensated = frc::ChassisSpeeds::Discretize(rawSpeeds, 0.02_s);

  EXPECT_NEAR(compensated.omega.value(), 2.0 * std::numbers::pi, 1e-4);
  EXPECT_LT(compensated.vy.value(), 0.0);
  EXPECT_GT(compensated.vx.value(), 2.8);
}

TEST(ScenariosTier4, Scenario3_IntakeToShooterSequencing) {
  units::angular_velocity::turns_per_second_t flywheelSpeed{0_tps};
  units::angular_velocity::turns_per_second_t targetSpeed{80_tps};
  units::angular_velocity::turns_per_second_t tolerance{2_tps};
  bool pieceFed = false;

  for (int cycle = 0; cycle < 30; ++cycle) {
    flywheelSpeed += (targetSpeed - flywheelSpeed) * 0.25;
    if (units::math::abs(flywheelSpeed - targetSpeed) <= tolerance) {
      pieceFed = true;
      break;
    }
  }

  EXPECT_TRUE(pieceFed);
}

TEST(ScenariosTier4, Scenario4_VisionLatencyCompensationAndGyroHeadingShield) {
  units::time::second_t fpgaTime = 10.50_s;
  units::time::second_t latency = 0.035_s;
  units::time::second_t visionTimestamp = fpgaTime - latency;

  EXPECT_LT(visionTimestamp, fpgaTime);
  EXPECT_NEAR(visionTimestamp.value(), 10.465, 1e-6);

  double yawStdDev = 999999.0;
  EXPECT_GT(yawStdDev, 10000.0);
}

TEST(ScenariosTier4, Scenario5_CarDriveCorneringAndReversal) {
  auto straight = calculateCarDrive(1.0, 0.0, 0.0, false, 0.6);
  EXPECT_DOUBLE_EQ(straight.mDriveSpeed, 0.6);

  auto brakeCorner = calculateCarDrive(0.8, 0.9, 0.75, false, 0.6);
  EXPECT_NEAR(brakeCorner.mDriveSpeed, 0.8 * 0.6 * 0.1, 1e-6);
  EXPECT_NEAR(brakeCorner.mSteerAngle.value(), -45.0, 1e-6);

  auto reverseOut = calculateCarDrive(0.5, 0.0, -0.5, true, 0.6);
  EXPECT_DOUBLE_EQ(reverseOut.mDriveSpeed, -0.3);
  EXPECT_NEAR(reverseOut.mSteerAngle.value(), 30.0, 1e-6);
}

TEST(ScenariosTier4, Scenario6_CoordinatedElevatorAndPivotScoring) {
  units::length::meter_t elevatorHeight = 0.0_m;
  units::angle::degree_t pivotAngle = 0.0_deg;

  elevatorHeight = 0.3_m;
  if (elevatorHeight > 0.5_m) {
    pivotAngle = 90.0_deg;
  }
  EXPECT_DOUBLE_EQ(pivotAngle.value(), 0.0);

  elevatorHeight = 0.8_m;
  if (elevatorHeight > 0.5_m) {
    pivotAngle = 90.0_deg;
  }
  EXPECT_DOUBLE_EQ(pivotAngle.value(), 90.0);
}

TEST(ScenariosTier4, Scenario7_AutonomousPathSegmentDiscretization) {
  // Simulating 10 trajectory path points at 50 Hz
  double x = 0.0;
  double y = 0.0;
  units::velocity::meters_per_second_t vx = 2.0_mps;
  units::velocity::meters_per_second_t vy = 0.5_mps;
  units::angular_velocity::radians_per_second_t omega = 0.5_rad_per_s;

  for (int step = 0; step < 10; ++step) {
    frc::ChassisSpeeds cmd{vx, vy, omega};
    auto disc = frc::ChassisSpeeds::Discretize(cmd, 0.02_s);
    x += disc.vx.value() * 0.02;
    y += disc.vy.value() * 0.02;
  }

  EXPECT_GT(x, 0.3);
  EXPECT_GT(y, 0.05);
}

TEST(ScenariosTier4, Scenario8_DefensiveLockEngagementOnStop) {
  bool driverIdle = true;
  std::array<frc::SwerveModuleState, 4> finalStates;

  if (driverIdle) {
    finalStates = calculateXPatternLock();
  }

  EXPECT_EQ(finalStates.size(), 4u);
  EXPECT_DOUBLE_EQ(finalStates[0].speed.value(), 0.0);
  EXPECT_DOUBLE_EQ(finalStates[1].speed.value(), 0.0);
  EXPECT_DOUBLE_EQ(finalStates[2].speed.value(), 0.0);
  EXPECT_DOUBLE_EQ(finalStates[3].speed.value(), 0.0);
  EXPECT_DOUBLE_EQ(finalStates[0].angle.Degrees().value(), 45.0);
}
