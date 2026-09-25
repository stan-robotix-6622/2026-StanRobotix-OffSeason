// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/nt/StructTopic.hpp>

#include <memory>
#include <string>

#include <wpi/units/angular_velocity.hpp>

#include <limelight/PoseEstimate.h>
#include <limelight/Limelight.h>

class Limelight {
 public:
	explicit Limelight(std::string_view iName);

	std::optional<wpi::math::Pose2d> getPoseEstimation(wpi::math::Pose2d iCurrentRobotPose, wpi::units::radians_per_second_t iRobotRotationalVelocity, bool iMegaTag2);

	void setCameraPosition(wpi::units::meter_t forward, wpi::units::meter_t right, wpi::units::meter_t up, wpi::units::degree_t roll, wpi::units::degree_t pitch, wpi::units::degree_t yaw);

 private:
	limelight::PoseEstimate mLimelightPoseEstimate;
	limelight::Limelight* mLimelight;
	bool mRejectCameraUpdate;

	std::string mName;
	wpi::nt::NetworkTableInstance inst = wpi::nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<wpi::nt::NetworkTable> mNTLimelightTable;
	wpi::nt::StructPublisher<wpi::math::Pose2d> mPoseEstimatorPublisher;
};
