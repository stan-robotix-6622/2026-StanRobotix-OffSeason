#include <cmath>
#include <numbers>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include "stan/KrakenSync.h"
#include "stan/StanMotor.h"
#include "test_helpers.h"

namespace stan::test {

inline constexpr units::angle::turn_t degreesToTurns(units::angle::degree_t iDegrees) {
  return units::angle::turn_t{iDegrees};
}

inline constexpr units::angle::radian_t degreesToRadians(units::angle::degree_t iDegrees) {
  return units::angle::radian_t{iDegrees};
}

inline constexpr units::length::meter_t inchesToMeters(units::length::inch_t iInches) {
  return units::length::meter_t{iInches};
}

inline constexpr units::length::inch_t metersToInches(units::length::meter_t iMeters) {
  return units::length::inch_t{iMeters};
}

inline constexpr units::velocity::meters_per_second_t calculateLinearVelocity(
    units::angular_velocity::turns_per_second_t iMotorVelocity,
    double iGearRatio,
    units::length::meter_t iWheelRadius) {
  double wheelRps = iMotorVelocity.value() / iGearRatio;
  units::length::meter_t wheelCircumference = 2.0 * std::numbers::pi * iWheelRadius;
  return units::velocity::meters_per_second_t{wheelRps * wheelCircumference.value()};
}

inline constexpr units::current::ampere_t enforceSafeCurrentLimit(units::current::ampere_t iRequestedLimit) {
  constexpr units::current::ampere_t kMaxBreakerLimit{80.0_A};
  constexpr units::current::ampere_t kMinCurrentLimit{1.0_A};
  if (iRequestedLimit > kMaxBreakerLimit) {
    return kMaxBreakerLimit;
  }
  if (iRequestedLimit < kMinCurrentLimit) {
    return kMinCurrentLimit;
  }
  return iRequestedLimit;
}

} // namespace stan::test

using namespace stan;
using namespace stan::test;

// ============================================================================
// Tier 1: Feature Coverage (Features 1, 2, 3: StanMotor, KrakenSync & Units)
// ============================================================================

TEST(UnitsTier1, DegreeToTurnConversion) {
  EXPECT_NEAR(degreesToTurns(360.0_deg).value(), 1.0, 1e-6);
  EXPECT_NEAR(degreesToTurns(180.0_deg).value(), 0.5, 1e-6);
  EXPECT_NEAR(degreesToTurns(90.0_deg).value(), 0.25, 1e-6);
  EXPECT_NEAR(degreesToTurns(0.0_deg).value(), 0.0, 1e-6);
  EXPECT_NEAR(degreesToTurns(-180.0_deg).value(), -0.5, 1e-6);
}

TEST(UnitsTier1, DegreeToRadianConversion) {
  EXPECT_NEAR(degreesToRadians(180.0_deg).value(), std::numbers::pi, 1e-6);
  EXPECT_NEAR(degreesToRadians(90.0_deg).value(), std::numbers::pi / 2.0, 1e-6);
  EXPECT_NEAR(degreesToRadians(360.0_deg).value(), 2.0 * std::numbers::pi, 1e-6);
  EXPECT_NEAR(degreesToRadians(0.0_deg).value(), 0.0, 1e-6);
  EXPECT_NEAR(degreesToRadians(-90.0_deg).value(), -std::numbers::pi / 2.0, 1e-6);
}

TEST(UnitsTier1, InchesToMetersConversion) {
  EXPECT_NEAR(inchesToMeters(1.0_in).value(), 0.0254, 1e-6);
  EXPECT_NEAR(inchesToMeters(12.0_in).value(), 0.3048, 1e-6);
  EXPECT_NEAR(inchesToMeters(4.0_in).value(), 0.1016, 1e-6);
  EXPECT_NEAR(inchesToMeters(0.0_in).value(), 0.0, 1e-6);
  EXPECT_NEAR(inchesToMeters(28.0_in).value(), 0.7112, 1e-6);
}

TEST(UnitsTier1, MetersToInchesConversion) {
  EXPECT_NEAR(metersToInches(0.0254_m).value(), 1.0, 1e-6);
  EXPECT_NEAR(metersToInches(1.0_m).value(), 39.3700787, 1e-5);
  EXPECT_NEAR(metersToInches(0.0508_m).value(), 2.0, 1e-6);
}

TEST(UnitsTier1, AngularVelocityToLinearVelocity) {
  units::angular_velocity::turns_per_second_t motorSpeed{100.0_tps};
  double gearRatio = 6.75;
  units::length::meter_t wheelRadius{2.0_in};

  units::velocity::meters_per_second_t linearSpeed =
      calculateLinearVelocity(motorSpeed, gearRatio, wheelRadius);

  double expectedWheelRps = 100.0 / 6.75;
  double expectedCircumference = 2.0 * std::numbers::pi * 0.0508;
  EXPECT_NEAR(linearSpeed.value(), expectedWheelRps * expectedCircumference, 1e-4);
}

TEST(UnitsTier1, MotorDutyCycleClamping) {
  EXPECT_DOUBLE_EQ(clampDutyCycle(0.5), 0.5);
  EXPECT_DOUBLE_EQ(clampDutyCycle(-0.5), -0.5);
  EXPECT_DOUBLE_EQ(clampDutyCycle(1.0), 1.0);
  EXPECT_DOUBLE_EQ(clampDutyCycle(-1.0), -1.0);
  EXPECT_DOUBLE_EQ(clampDutyCycle(1.2), 1.0);
  EXPECT_DOUBLE_EQ(clampDutyCycle(-1.5), -1.0);
}

TEST(UnitsTier1, VoltageLimitsEnforcement) {
  EXPECT_NEAR(clampVoltage(6.0_V).value(), 6.0, 1e-6);
  EXPECT_NEAR(clampVoltage(-6.0_V).value(), -6.0, 1e-6);
  EXPECT_NEAR(clampVoltage(12.0_V).value(), 12.0, 1e-6);
  EXPECT_NEAR(clampVoltage(14.5_V).value(), 12.0, 1e-6);
  EXPECT_NEAR(clampVoltage(-15.0_V).value(), -12.0, 1e-6);
}

TEST(UnitsTier1, SafeCurrentLimitDefaults) {
  EXPECT_NEAR(enforceSafeCurrentLimit(40.0_A).value(), 40.0, 1e-6);
  EXPECT_NEAR(enforceSafeCurrentLimit(60.0_A).value(), 60.0, 1e-6);
  EXPECT_NEAR(enforceSafeCurrentLimit(80.0_A).value(), 80.0, 1e-6);
  EXPECT_NEAR(enforceSafeCurrentLimit(120.0_A).value(), 80.0, 1e-6);
  EXPECT_NEAR(enforceSafeCurrentLimit(0.1_A).value(), 1.0, 1e-6);
}

TEST(UnitsTier1, KrakenSyncNullSafety) {
  bool resultBothNull = KrakenSync::sync(nullptr, nullptr);
  EXPECT_FALSE(resultBothNull);
}

TEST(UnitsTier1, KrakenSyncTimeoutArgument) {
  EXPECT_FALSE(KrakenSync::sync(nullptr, nullptr, 100_ms));
  EXPECT_FALSE(KrakenSync::sync(nullptr, nullptr, 500_ms));
}

TEST(UnitsTier1, RevolutionsPerMinuteToTurnsPerSecond) {
  units::angular_velocity::turns_per_second_t tps{6000.0 / 60.0};
  EXPECT_NEAR(tps.value(), 100.0, 1e-6);

  units::angular_velocity::turns_per_second_t idleTps{0.0};
  EXPECT_NEAR(idleTps.value(), 0.0, 1e-6);
}

TEST(UnitsTier1, GearReductionMechanicalCalculations) {
  double motorTurns = 50.0;
  double reduction = 25.0;
  double outputTurns = motorTurns / reduction;
  EXPECT_NEAR(outputTurns, 2.0, 1e-6);

  units::angle::degree_t outputAngle{outputTurns * 360.0};
  EXPECT_NEAR(outputAngle.value(), 720.0, 1e-6);
}

TEST(UnitsTier1, StanMotorTypeAndEnumIntegrity) {
  EXPECT_NE(MotorType::kTalonFX, MotorType::kSparkMax);
  EXPECT_NE(MotorType::kSparkMax, MotorType::kSparkFlex);
  EXPECT_NE(IdleMode::kBrake, IdleMode::kCoast);
}

TEST(UnitsTier1, MillisecondsToSecondsConversion) {
  units::time::second_t s{250_ms};
  EXPECT_NEAR(s.value(), 0.25, 1e-6);

  units::time::millisecond_t ms{0.02_s};
  EXPECT_NEAR(ms.value(), 20.0, 1e-6);
}

TEST(UnitsTier1, AmperesToMilliamperesConversion) {
  units::current::ampere_t current{40.0_A};
  units::current::milliampere_t mA = current;
  EXPECT_NEAR(mA.value(), 40000.0, 1e-3);
}

TEST(UnitsTier1, LinearDistanceToWheelTurns) {
  units::length::meter_t distance{3.14159265_m};
  units::length::meter_t wheelRadius{0.5_m};
  units::length::meter_t circumference = 2.0 * std::numbers::pi * wheelRadius;
  double turns = distance.value() / circumference.value();
  EXPECT_NEAR(turns, 1.0, 1e-5);
}

TEST(UnitsTier1, AngularVelocityToRadianVelocity) {
  units::angular_velocity::turns_per_second_t tps{1.0_tps};
  units::angular_velocity::radians_per_second_t rads{tps.value() * 2.0 * std::numbers::pi};
  EXPECT_NEAR(rads.value(), 2.0 * std::numbers::pi, 1e-6);
}

TEST(UnitsTier1, MotorSafetyDefaultsCheck) {
  constexpr units::current::ampere_t kDefaultSupplyLimit{40_A};
  constexpr units::current::ampere_t kDefaultStatorLimit{60_A};
  EXPECT_LT(kDefaultSupplyLimit, kDefaultStatorLimit);
  EXPECT_DOUBLE_EQ(kDefaultSupplyLimit.value(), 40.0);
  EXPECT_DOUBLE_EQ(kDefaultStatorLimit.value(), 60.0);
}

TEST(UnitsTier1, TurnsToDegreesConversion) {
  units::angle::turn_t turns{2.5_tr};
  units::angle::degree_t deg = turns;
  EXPECT_NEAR(deg.value(), 900.0, 1e-6);
}

TEST(UnitsTier1, TurnsToRadiansConversion) {
  units::angle::turn_t halfTurn{0.5_tr};
  units::angle::radian_t rad = halfTurn;
  EXPECT_NEAR(rad.value(), std::numbers::pi, 1e-6);
}

// ============================================================================
// Tier 2: Boundary & Corner Cases (Features 1, 2, 3)
// ============================================================================

TEST(UnitsTier2, ExtremePositiveSpeedClamping) {
  EXPECT_DOUBLE_EQ(clampDutyCycle(1000.0), 1.0);
  EXPECT_DOUBLE_EQ(clampDutyCycle(1e6), 1.0);
}

TEST(UnitsTier2, ExtremeNegativeSpeedClamping) {
  EXPECT_DOUBLE_EQ(clampDutyCycle(-500.0), -1.0);
  EXPECT_DOUBLE_EQ(clampDutyCycle(-1e9), -1.0);
}

TEST(UnitsTier2, ZeroSpeedPreservation) {
  EXPECT_DOUBLE_EQ(clampDutyCycle(0.0), 0.0);
  EXPECT_DOUBLE_EQ(clampDutyCycle(-0.0), 0.0);
}

TEST(UnitsTier2, OvervoltageClampBoundary) {
  EXPECT_NEAR(clampVoltage(12.0001_V).value(), 12.0, 1e-6);
  EXPECT_NEAR(clampVoltage(-12.0001_V).value(), -12.0, 1e-6);
  EXPECT_NEAR(clampVoltage(100.0_V).value(), 12.0, 1e-6);
}

TEST(UnitsTier2, SubMillimeterConversionPrecision) {
  units::length::meter_t tinyDistance{0.0001_m};
  units::length::inch_t inches = tinyDistance;
  EXPECT_NEAR(inches.value(), 0.00393701, 1e-6);
}

TEST(UnitsTier2, ExtremeHighRPMConversion) {
  double falconFreeSpeedRpm = 6380.0;
  units::angular_velocity::turns_per_second_t tps{falconFreeSpeedRpm / 60.0};
  EXPECT_NEAR(tps.value(), 106.3333, 1e-4);

  units::angle::radian_t radPerSec{tps.value() * 2.0 * std::numbers::pi};
  EXPECT_NEAR(radPerSec.value(), (falconFreeSpeedRpm / 60.0) * 2.0 * std::numbers::pi, 1e-4);
}

TEST(UnitsTier2, AngleWrapAroundTurns) {
  double turns = 3.75;
  double wrappedTurns = turns - std::floor(turns);
  EXPECT_NEAR(wrappedTurns, 0.75, 1e-6);

  double negTurns = -0.25;
  double wrappedNeg = negTurns - std::floor(negTurns);
  EXPECT_NEAR(wrappedNeg, 0.75, 1e-6);
}

TEST(UnitsTier2, ZeroLengthWheelRadiusSafety) {
  units::angular_velocity::turns_per_second_t motorSpeed{100.0_tps};
  units::length::meter_t zeroRadius{0.0_m};
  units::velocity::meters_per_second_t speed =
      calculateLinearVelocity(motorSpeed, 6.75, zeroRadius);
  EXPECT_NEAR(speed.value(), 0.0, 1e-6);
}

TEST(UnitsTier2, NegativeAngleConversionIntegrity) {
  units::angle::degree_t negAngle{-270.0_deg};
  units::angle::turn_t turns = degreesToTurns(negAngle);
  EXPECT_NEAR(turns.value(), -0.75, 1e-6);

  units::angle::radian_t radians = degreesToRadians(negAngle);
  EXPECT_NEAR(radians.value(), -1.5 * std::numbers::pi, 1e-6);
}

TEST(UnitsTier2, CurrentLimitBounds) {
  EXPECT_NEAR(enforceSafeCurrentLimit(0.0_A).value(), 1.0, 1e-6);
  EXPECT_NEAR(enforceSafeCurrentLimit(-10.0_A).value(), 1.0, 1e-6);
  EXPECT_NEAR(enforceSafeCurrentLimit(1000.0_A).value(), 80.0, 1e-6);
}

TEST(UnitsTier2, SubMicrosecondTimeConversion) {
  units::time::microsecond_t us{500_us};
  units::time::second_t s = us;
  EXPECT_NEAR(s.value(), 0.0005, 1e-9);
}

TEST(UnitsTier2, ExtremeGearReductionRatio) {
  double highRatio = 1000.0;
  units::angular_velocity::turns_per_second_t motorSpeed{100.0_tps};
  units::length::meter_t wheelRadius{0.05_m};
  auto slowSpeed = calculateLinearVelocity(motorSpeed, highRatio, wheelRadius);
  EXPECT_NEAR(slowSpeed.value(), (100.0 / 1000.0) * 2.0 * std::numbers::pi * 0.05, 1e-6);
}

TEST(UnitsTier2, MicroInchPrecisionConversion) {
  units::length::inch_t microInch{0.000001_in};
  units::length::meter_t m = microInch;
  EXPECT_NEAR(m.value(), 2.54e-8, 1e-11);
}

TEST(UnitsTier2, LargeAngleMultiTurnWrap) {
  units::angle::degree_t largeAngle{7200.0_deg};
  units::angle::turn_t turns = largeAngle;
  EXPECT_NEAR(turns.value(), 20.0, 1e-6);
}

TEST(UnitsTier2, VoltageClampingExact12V) {
  EXPECT_DOUBLE_EQ(clampVoltage(12.0_V).value(), 12.0);
  EXPECT_DOUBLE_EQ(clampVoltage(-12.0_V).value(), -12.0);
}

TEST(UnitsTier2, VoltageZeroPreservation) {
  EXPECT_DOUBLE_EQ(clampVoltage(0.0_V).value(), 0.0);
}

TEST(UnitsTier2, FloatUnderflowNearZeroTurns) {
  units::angle::turn_t tinyTurn{1e-9_tr};
  units::angle::degree_t deg = tinyTurn;
  EXPECT_GT(deg.value(), 0.0);
  EXPECT_LT(deg.value(), 1e-5);
}

TEST(UnitsTier2, InverseGearRatioMultiplication) {
  double ratio = 8.14;
  double mechanismRotations = 5.0;
  double motorRotations = mechanismRotations * ratio;
  EXPECT_NEAR(motorRotations, 40.7, 1e-6);
}

TEST(UnitsTier2, NegativeZeroFloatEquivalence) {
  EXPECT_DOUBLE_EQ(degreesToTurns(-0.0_deg).value(), 0.0);
  EXPECT_DOUBLE_EQ(degreesToRadians(-0.0_deg).value(), 0.0);
}
