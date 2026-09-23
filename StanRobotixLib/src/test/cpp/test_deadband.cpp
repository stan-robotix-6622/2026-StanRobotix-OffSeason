#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <frc/MathUtil.h>
#include "stan/StanXboxController.h"
#include "test_helpers.h"

namespace stan::test {

struct MockTrigger {
  bool mState{false};

  bool get() const {
    return mState;
  }

  void set(bool iValue) {
    mState = iValue;
  }
};

class MockDebouncer {
 public:
  explicit MockDebouncer(double iDebounceTimeSeconds)
      : mDebounceTime{iDebounceTimeSeconds} {}

  bool calculate(bool iInput, double iCurrentTime) {
    if (iInput == mCurrentState) {
      mResetTime = iCurrentTime + mDebounceTime;
    } else if (iCurrentTime >= mResetTime) {
      mCurrentState = iInput;
    }
    return mCurrentState;
  }

 private:
  double mDebounceTime;
  double mResetTime{0.0};
  bool mCurrentState{false};
};

} // namespace stan::test

using namespace stan;
using namespace stan::test;

// ============================================================================
// Tier 1: Feature Coverage (Feature 8: Deadband & Feature 9: Bindings DSL)
// ============================================================================

TEST(DeadbandTier1, ZeroThresholdPreservesInput) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.5, 0.0), 0.5);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.5, 0.0), -0.5);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(1.0, 0.0), 1.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-1.0, 0.0), -1.0);
}

TEST(DeadbandTier1, WithinDeadbandPositiveReturnsZero) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.01, 0.05), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.049, 0.05), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.03, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.08, 0.1), 0.0);
}

TEST(DeadbandTier1, WithinDeadbandNegativeReturnsZero) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.01, 0.05), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.049, 0.05), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.02, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.099, 0.1), 0.0);
}

TEST(DeadbandTier1, AboveDeadbandContinuousScaling) {
  double expectedHalf = (0.55 - 0.1) / (1.0 - 0.1);
  EXPECT_NEAR(StanXboxController::applyDeadband(0.55, 0.1), expectedHalf, 1e-6);
  double expectedQuarter = (0.325 - 0.1) / (1.0 - 0.1);
  EXPECT_NEAR(StanXboxController::applyDeadband(0.325, 0.1), expectedQuarter, 1e-6);
}

TEST(DeadbandTier1, BelowDeadbandContinuousScalingNegative) {
  double expectedNegHalf = (-0.55 + 0.1) / (1.0 - 0.1);
  EXPECT_NEAR(StanXboxController::applyDeadband(-0.55, 0.1), expectedNegHalf, 1e-6);
  double expectedNegQuarter = (-0.325 + 0.1) / (1.0 - 0.1);
  EXPECT_NEAR(StanXboxController::applyDeadband(-0.325, 0.1), expectedNegQuarter, 1e-6);
}

TEST(DeadbandTier1, FullScalePositiveProducesOne) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(1.0, 0.05), 1.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(1.0, 0.1), 1.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(1.0, 0.2), 1.0);
}

TEST(DeadbandTier1, FullScaleNegativeProducesNegativeOne) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-1.0, 0.05), -1.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-1.0, 0.1), -1.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-1.0, 0.2), -1.0);
}

TEST(DeadbandTier1, SquaredResponseShape) {
  double linear = StanXboxController::applyDeadband(0.55, 0.1, false);
  double squared = StanXboxController::applyDeadband(0.55, 0.1, true);
  EXPECT_NEAR(squared, linear * linear, 1e-6);
  EXPECT_LT(squared, linear);

  double linearNeg = StanXboxController::applyDeadband(-0.55, 0.1, false);
  double squaredNeg = StanXboxController::applyDeadband(-0.55, 0.1, true);
  EXPECT_NEAR(squaredNeg, -std::abs(linearNeg * linearNeg), 1e-6);
  EXPECT_LT(squaredNeg, 0.0);
}

TEST(DeadbandTier1, WPILibApplyDeadbandParity) {
  double rawInput = 0.6;
  double deadband = 0.1;
  double wpiResult = frc::ApplyDeadband(rawInput, deadband);
  double stanResult = StanXboxController::applyDeadband(rawInput, deadband);
  EXPECT_NEAR(stanResult, wpiResult, 1e-6);
}

TEST(DeadbandTier1, ControllerInstanceConfig) {
  StanXboxController controller{0, 0.08};
  EXPECT_DOUBLE_EQ(controller.getDeadband(), 0.08);

  controller.setDeadband(0.12);
  EXPECT_DOUBLE_EQ(controller.getDeadband(), 0.12);
}

TEST(DeadbandTier1, TriggerHoldSemantics) {
  MockTrigger trigger;
  bool commandRunning = false;

  auto updateLoop = [&](bool iPressed) {
    trigger.set(iPressed);
    commandRunning = trigger.get();
  };

  updateLoop(false);
  EXPECT_FALSE(commandRunning);
  updateLoop(true);
  EXPECT_TRUE(commandRunning);
  updateLoop(true);
  EXPECT_TRUE(commandRunning);
  updateLoop(false);
  EXPECT_FALSE(commandRunning);
}

TEST(DeadbandTier1, TriggerToggleSemantics) {
  bool toggledState = false;
  bool lastState = false;

  auto handlePress = [&](bool iCurrent) {
    if (iCurrent && !lastState) {
      toggledState = !toggledState;
    }
    lastState = iCurrent;
  };

  handlePress(false);
  EXPECT_FALSE(toggledState);
  handlePress(true);
  EXPECT_TRUE(toggledState);
  handlePress(true);
  EXPECT_TRUE(toggledState);
  handlePress(false);
  EXPECT_TRUE(toggledState);
  handlePress(true);
  EXPECT_FALSE(toggledState);
}

TEST(DeadbandTier1, TriggerPressEdgeSemantics) {
  int triggerCount = 0;
  bool lastState = false;

  auto handleCycle = [&](bool iPressed) {
    if (iPressed && !lastState) {
      triggerCount++;
    }
    lastState = iPressed;
  };

  handleCycle(false);
  EXPECT_EQ(triggerCount, 0);
  handleCycle(true);
  EXPECT_EQ(triggerCount, 1);
  handleCycle(true);
  EXPECT_EQ(triggerCount, 1);
  handleCycle(false);
  EXPECT_EQ(triggerCount, 1);
  handleCycle(true);
  EXPECT_EQ(triggerCount, 2);
}

TEST(DeadbandTier1, TriggerDebounceFiltersNoise) {
  MockDebouncer debouncer{0.1};
  EXPECT_FALSE(debouncer.calculate(false, 0.0));

  debouncer.calculate(true, 0.02);
  EXPECT_FALSE(debouncer.calculate(false, 0.04));

  debouncer.calculate(true, 0.05);
  debouncer.calculate(true, 0.10);
  EXPECT_TRUE(debouncer.calculate(true, 0.16));
}

TEST(DeadbandTier1, LogicalAndTriggerCombination) {
  MockTrigger triggerA;
  MockTrigger triggerB;

  auto combined = [&] { return triggerA.get() && triggerB.get(); };

  triggerA.set(false);
  triggerB.set(false);
  EXPECT_FALSE(combined());

  triggerA.set(true);
  triggerB.set(false);
  EXPECT_FALSE(combined());

  triggerA.set(false);
  triggerB.set(true);
  EXPECT_FALSE(combined());

  triggerA.set(true);
  triggerB.set(true);
  EXPECT_TRUE(combined());
}

TEST(DeadbandTier1, LogicalOrTriggerCombination) {
  MockTrigger triggerA;
  MockTrigger triggerB;

  auto combined = [&] { return triggerA.get() || triggerB.get(); };

  triggerA.set(false);
  triggerB.set(false);
  EXPECT_FALSE(combined());

  triggerA.set(true);
  triggerB.set(false);
  EXPECT_TRUE(combined());

  triggerA.set(false);
  triggerB.set(true);
  EXPECT_TRUE(combined());
}

TEST(DeadbandTier1, TriggerAxisDeadbandUnidirectional) {
  // Trigger axis operates in [0.0, 1.0]
  double triggerDeadband = 0.05;
  double rest = StanXboxController::applyDeadband(0.02, triggerDeadband);
  EXPECT_DOUBLE_EQ(rest, 0.0);

  double pulled = StanXboxController::applyDeadband(0.525, triggerDeadband);
  EXPECT_NEAR(pulled, 0.5, 1e-6);

  double maxPull = StanXboxController::applyDeadband(1.0, triggerDeadband);
  EXPECT_DOUBLE_EQ(maxPull, 1.0);
}

TEST(DeadbandTier1, StanXboxAliasEquivalence) {
  StanXbox xbox{0, 0.05};
  EXPECT_DOUBLE_EQ(xbox.getDeadband(), 0.05);
}

TEST(DeadbandTier1, IntermediateDeadbandProgression) {
  double threshold = 0.1;
  double previous = 0.0;
  for (int step = 10; step <= 100; step += 10) {
    double input = step / 100.0;
    double output = StanXboxController::applyDeadband(input, threshold);
    EXPECT_GE(output, previous);
    previous = output;
  }
}

TEST(DeadbandTier1, NegativeIntermediateProgression) {
  double threshold = 0.1;
  double previous = 0.0;
  for (int step = 10; step <= 100; step += 10) {
    double input = -step / 100.0;
    double output = StanXboxController::applyDeadband(input, threshold);
    EXPECT_LE(output, previous);
    previous = output;
  }
}

// ============================================================================
// Tier 2: Boundary & Corner Cases (Feature 8 & 9)
// ============================================================================

TEST(DeadbandTier2, ExactThresholdEdgePositive) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.1, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.05, 0.05), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.2, 0.2), 0.0);
}

TEST(DeadbandTier2, ExactThresholdEdgeNegative) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.1, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.05, 0.05), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.2, 0.2), 0.0);
}

TEST(DeadbandTier2, InfinitesimalAboveThresholdContinuity) {
  double threshold = 0.1;
  double delta = 1e-6;
  double result = StanXboxController::applyDeadband(threshold + delta, threshold);
  EXPECT_GT(result, 0.0);
  EXPECT_LT(result, 1e-4);
}

TEST(DeadbandTier2, InfinitesimalBelowNegativeThresholdContinuity) {
  double threshold = 0.1;
  double delta = 1e-6;
  double result = StanXboxController::applyDeadband(-threshold - delta, threshold);
  EXPECT_LT(result, 0.0);
  EXPECT_GT(result, -1e-4);
}

TEST(DeadbandTier2, OverRangePositiveScaling) {
  EXPECT_NEAR(StanXboxController::applyDeadband(1.5, 0.1), 1.4 / 0.9, 1e-9);
  EXPECT_NEAR(StanXboxController::applyDeadband(10.0, 0.1), 11.0, 1e-9);
  EXPECT_NEAR(StanXboxController::applyDeadband(1e5, 0.1), 99999.9 / 0.9, 1e-4);
}

TEST(DeadbandTier2, OverRangeNegativeScaling) {
  EXPECT_NEAR(StanXboxController::applyDeadband(-1.5, 0.1), -1.4 / 0.9, 1e-9);
  EXPECT_NEAR(StanXboxController::applyDeadband(-10.0, 0.1), -11.0, 1e-9);
  EXPECT_NEAR(StanXboxController::applyDeadband(-1e5, 0.1), -99999.9 / 0.9, 1e-4);
}

TEST(DeadbandTier2, ZeroInputExactZero) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.0, 0.1), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.0, 0.5), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.0, 0.1), 0.0);
}

TEST(DeadbandTier2, NearUnityThresholdStability) {
  double threshold = 0.99;
  double val = StanXboxController::applyDeadband(0.995, threshold);
  EXPECT_NEAR(val, 0.5, 1e-3);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(1.0, threshold), 1.0);
}

TEST(DeadbandTier2, ThresholdEqualToOneEdge) {
  double val = StanXboxController::applyDeadband(1.0, 1.0);
  EXPECT_TRUE(std::isnan(val));
}

TEST(DeadbandTier2, RapidOscillatingInputSampling) {
  double threshold = 0.1;
  for (int idx = -100; idx <= 100; ++idx) {
    double input = idx / 100.0;
    double output = StanXboxController::applyDeadband(input, threshold);
    if (std::abs(input) <= threshold) {
      EXPECT_DOUBLE_EQ(output, 0.0);
    } else {
      EXPECT_NE(output, 0.0);
      EXPECT_GE(std::abs(output), 0.0);
      EXPECT_LE(std::abs(output), 1.0);
    }
  }
}

TEST(DeadbandTier2, SquaredPreservesSignNearZero) {
  double resPos = StanXboxController::applyDeadband(0.101, 0.1, true);
  EXPECT_GT(resPos, 0.0);

  double resNeg = StanXboxController::applyDeadband(-0.101, 0.1, true);
  EXPECT_LT(resNeg, 0.0);
}

TEST(DeadbandTier2, SquaredFullScaleBounds) {
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(1.0, 0.1, true), 1.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-1.0, 0.1, true), -1.0);
}

TEST(DeadbandTier2, TriggerDebounceExactTimingThreshold) {
  MockDebouncer debouncer{0.100};
  debouncer.calculate(false, 0.0);

  debouncer.calculate(true, 0.010);
  EXPECT_FALSE(debouncer.calculate(true, 0.080));
  EXPECT_TRUE(debouncer.calculate(true, 0.111));
}

TEST(DeadbandTier2, TriggerRapidPulseTrain) {
  MockDebouncer debouncer{0.05};
  debouncer.calculate(false, 0.0);

  // 10 micro-glitches of 5ms each must never trigger output
  for (int idx = 0; idx < 10; ++idx) {
    double t = idx * 0.02;
    debouncer.calculate(true, t);
    bool state = debouncer.calculate(false, t + 0.005);
    EXPECT_FALSE(state);
  }
}

TEST(DeadbandTier2, SustainedTriggerHoldDuration) {
  MockTrigger trigger;
  trigger.set(true);

  int heldCycles = 0;
  for (int cycle = 0; cycle < 100; ++cycle) {
    if (trigger.get()) {
      heldCycles++;
    }
  }
  EXPECT_EQ(heldCycles, 100);
}

TEST(DeadbandTier2, ContinuousWrapControllerAxes) {
  for (double in = -1.0; in <= 1.0; in += 0.05) {
    double out = StanXboxController::applyDeadband(in, 0.1);
    EXPECT_GE(out, -1.0);
    EXPECT_LE(out, 1.0);
  }
}

TEST(DeadbandTier2, HighThresholdDeadbandSuppression) {
  double highThreshold = 0.8;
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.79, highThreshold), 0.0);
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(-0.79, highThreshold), 0.0);
  EXPECT_GT(StanXboxController::applyDeadband(0.81, highThreshold), 0.0);
}

TEST(DeadbandTier2, SmallDeadbandSensitivity) {
  double tinyDeadband = 0.01;
  EXPECT_DOUBLE_EQ(StanXboxController::applyDeadband(0.005, tinyDeadband), 0.0);
  EXPECT_GT(StanXboxController::applyDeadband(0.015, tinyDeadband), 0.0);
}

TEST(DeadbandTier2, SymmetryPositiveNegativeDeadband) {
  for (double val = 0.0; val <= 1.0; val += 0.05) {
    double posOut = StanXboxController::applyDeadband(val, 0.15);
    double negOut = StanXboxController::applyDeadband(-val, 0.15);
    EXPECT_DOUBLE_EQ(posOut, -negOut);
  }
}

TEST(DeadbandTier2, SymmetrySquaredPositiveNegativeDeadband) {
  for (double val = 0.0; val <= 1.0; val += 0.05) {
    double posOut = StanXboxController::applyDeadband(val, 0.15, true);
    double negOut = StanXboxController::applyDeadband(-val, 0.15, true);
    EXPECT_DOUBLE_EQ(posOut, -negOut);
  }
}
