#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>
#include <string_view>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

namespace stan::test {

inline double calculateDeadband(double iInput, double iThreshold, bool iSquared = false) {
  double threshold = std::clamp(iThreshold, 0.0, 0.999);
  if (std::abs(iInput) <= threshold) {
    return 0.0;
  }
  double scaled = (iInput > 0.0) ? (iInput - threshold) / (1.0 - threshold)
                                 : (iInput + threshold) / (1.0 - threshold);
  scaled = std::clamp(scaled, -1.0, 1.0);
  if (iSquared) {
    return scaled * std::abs(scaled);
  }
  return scaled;
}

inline constexpr units::voltage::volt_t clampVoltage(
    units::voltage::volt_t iVoltage,
    units::voltage::volt_t iMaxVoltage = 12.0_V) {
  if (iVoltage > iMaxVoltage) {
    return iMaxVoltage;
  }
  if (iVoltage < -iMaxVoltage) {
    return -iMaxVoltage;
  }
  return iVoltage;
}

inline constexpr double clampDutyCycle(double iSpeed) {
  return std::clamp(iSpeed, -1.0, 1.0);
}

class DiscreteSlewRateLimiter {
 public:
  explicit DiscreteSlewRateLimiter(units::meters_per_second_squared_t iRateLimit)
      : mRateLimit{iRateLimit} {}

  void reset(units::meters_per_second_t iValue) {
    mValue = iValue;
  }

  units::meters_per_second_t calculate(units::meters_per_second_t iTarget, units::second_t iDt = 0.02_s) {
    units::meters_per_second_t maxChange = mRateLimit * iDt;
    units::meters_per_second_t delta = iTarget - mValue;
    if (delta > maxChange) {
      mValue += maxChange;
    } else if (delta < -maxChange) {
      mValue -= maxChange;
    } else {
      mValue = iTarget;
    }
    return mValue;
  }

 private:
  units::meters_per_second_squared_t mRateLimit;
  units::meters_per_second_t mValue{0.0_mps};
};

struct CarDriveOutputs {
  double mDriveSpeed{0.0};
  units::angle::degree_t mSteerAngle{0.0_deg};
};

inline CarDriveOutputs calculateCarDrive(
    double iThrottle,
    double iBrake,
    double iSteer,
    bool iReverse,
    double iSpeedScale = 0.5,
    units::angle::degree_t iMaxSteerAngle = 60.0_deg) {
  double throttle = std::clamp(iThrottle, 0.0, 1.0);
  double brake = std::clamp(iBrake, 0.0, 1.0);
  double speed = (throttle * iSpeedScale) * (1.0 - brake);
  if (speed < 0.0) {
    speed = 0.0;
  }
  if (iReverse) {
    speed = -speed;
  }
  units::angle::degree_t steerAngle = -std::clamp(iSteer, -1.0, 1.0) * iMaxSteerAngle;
  return {speed, steerAngle};
}

inline std::array<frc::SwerveModuleState, 4> calculateXPatternLock() {
  return {
      frc::SwerveModuleState{0.0_mps, frc::Rotation2d{45.0_deg}},
      frc::SwerveModuleState{0.0_mps, frc::Rotation2d{-45.0_deg}},
      frc::SwerveModuleState{0.0_mps, frc::Rotation2d{-45.0_deg}},
      frc::SwerveModuleState{0.0_mps, frc::Rotation2d{45.0_deg}},
  };
}

struct VisionPoseEstimate {
  frc::Pose2d pose;
  units::time::second_t timestampSeconds{0_s};
  int tagCount{0};
  bool hasData{false};
};

inline bool evaluateVisionMeasurement(
    const VisionPoseEstimate& iEstimate,
    units::angular_velocity::degrees_per_second_t iRobotRotationalVelocity,
    bool iMegaTag2) {
  if (!iEstimate.hasData) {
    return false;
  }
  if (units::math::abs(iRobotRotationalVelocity) > 180.0_deg_per_s) {
    return false;
  }
  if (iEstimate.tagCount == 0) {
    return false;
  }
  if (iEstimate.pose == frc::Pose2d(0.0_m, 0.0_m, 0.0_rad)) {
    return false;
  }
  if (!iMegaTag2 && iEstimate.tagCount < 2) {
    return false;
  }
  return true;
}

inline double calculatePIDOutput(
    double iSetpoint,
    double iMeasurement,
    double ikP,
    double ikI,
    double ikD,
    double ikS,
    double ikV,
    double iDt,
    double& ioIntegral,
    double& ioPrevError) {
  double error = iSetpoint - iMeasurement;
  ioIntegral += error * iDt;
  double derivative = (iDt > 0.0) ? (error - ioPrevError) / iDt : 0.0;
  ioPrevError = error;

  double pTerm = ikP * error;
  double iTerm = ikI * ioIntegral;
  double dTerm = ikD * derivative;

  double ffTerm = 0.0;
  if (std::abs(iSetpoint) > 1e-4) {
    double sign = (iSetpoint > 0.0) ? 1.0 : -1.0;
    ffTerm = (ikS * sign) + (ikV * iSetpoint);
  }

  return pTerm + iTerm + dTerm + ffTerm;
}

} // namespace stan::test
