// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Limelight.hpp"

#include "Constants.hpp"

Limelight::Limelight(std::string_view iName)
{
	mLimelight = new limelight::Limelight{iName};
	mNTLimelightTable = inst.GetTable("SmartDashboard/" + mName);
	mPoseEstimatorPublisher = mNTLimelightTable->GetStructTopic<wpi::math::Pose2d>("Pose Estimator").Publish();
}

std::optional<wpi::math::Pose2d> Limelight::getPoseEstimation(wpi::math::Pose2d iCurrentRobotPose, wpi::units::radians_per_second_t iRobotRotationalVelocity, bool iMegaTag2)
{
	// Update la rotation du robot pour la Limelight

	// limelight::(mName, iCurrentRobotPose.Rotation().Degrees().value(), 0, 0, 0, 0, 0);

	// if (iMegaTag2) {
	// 	mLimelightPoseEstimate = mLimelight->GetRobotPose();
	// }
	// else {
	// 	mLimelightPoseEstimate = limelight::getBotPoseEstimate_wpiBlue(mName);
	// }

	// // reject the camera update if the PoseEstimate is not valid
	// mRejectCameraUpdate = !limelight::validPoseEstimate(mLimelightPoseEstimate);

	// if (wpi::units::math::abs(iRobotRotationalVelocity) > 180_deg_per_s) {
	// 	mRejectCameraUpdate = true;
	// }
	// else if (mLimelightPoseEstimate.tagCount == 0) {
	// 	mRejectCameraUpdate = true;
	// }
	// else if (mLimelightPoseEstimate.pose == wpi::math::Pose2d(0_m, 0_m, 0_rad)) {
	// 	mRejectCameraUpdate = true;
	// }
	// else if (!iMegaTag2 && mLimelightPoseEstimate.tagCount < 2) {
	// 	mRejectCameraUpdate = true;
	// }

	// if (!mRejectCameraUpdate) {
	// 	// limelight::PrintPoseEstimate(mLimelightPoseEstimate);
	// 	mPoseEstimatorPublisher.Set(mLimelightPoseEstimate.pose);
	// 	return mLimelightPoseEstimate.pose;
	// }
	return {};
}

void Limelight::setCameraPosition(wpi::units::meter_t iForward, wpi::units::meter_t iRight, wpi::units::meter_t iUp, wpi::units::degree_t iRoll, wpi::units::degree_t iPitch, wpi::units::degree_t iYaw)
{
	// limelight::setCameraPose_RobotSpace(
	// 		mName,
	// 		iForward.value(),
	// 		iRight.value(),
	// 		iUp.value(),
	// 		iRoll.value(),
	// 		iPitch.value(),
	// 		iYaw.value());
}
