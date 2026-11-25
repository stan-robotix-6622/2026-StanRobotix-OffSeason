// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/math/estimator/SwerveDrivePoseEstimator.hpp>
#include <wpi/math/geometry/Pose2d.hpp>
#include <wpi/math/geometry/Translation2d.hpp>
#include <wpi/math/kinematics/ChassisVelocities.hpp>
#include <wpi/math/kinematics/SwerveDriveKinematics.hpp>
#include <wpi/math/kinematics/SwerveDriveOdometry.hpp>
#include <wpi/smartdashboard/Field2d.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/nt/NetworkTable.hpp>
#include <wpi/nt/NetworkTableInstance.hpp>
#include <wpi/nt/StructArrayTopic.hpp>
#include <wpi/nt/StructTopic.hpp>

#include <memory>
#include <string>

#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>

#include "subsystems/IMU.hpp"
#include "subsystems/Limelight.hpp"
#include "subsystems/SwerveModule.hpp"

class SubDrivetrain : public wpi::cmd::SubsystemBase {
 public:
	SubDrivetrain();

	void Periodic() override;
	// void InitSendable(wpi::util::SendableBuilder& builder) override;

	void ConfigurePathplanner();

	void mesureSwerveFeedforward(wpi::units::volt_t iDrivingVoltage, wpi::util::array<wpi::math::Rotation2d, 4> iDesiredHeadings);

	void setSwerveModuleVelocities(wpi::util::array<wpi::math::SwerveModuleVelocity, 4>);
	void refreshSwerveModules();
	wpi::util::array<wpi::math::SwerveModuleVelocity, 4> getSwerveModuleVelocities();
	wpi::util::array<wpi::math::SwerveModulePosition, 4> getSwerveModulePositions();

	// wpi::cmd::CommandPtr getFollowPathCommand(std::string iPathName);

	wpi::math::ChassisVelocities getRobotRelativeSpeeds();
	wpi::math::ChassisVelocities getFieldRelativeSpeeds();
	// wpi::cmd::CommandPtr getDriveCommand(std::function<double()> iXSupplier,
	// 																 std::function<double()> iYSupplier,
	// 																 std::function<double()> i0Supplier,
	// 																 std::function<double()> iSpeedModulationSupplier,
	// 																 bool iFieldRelative);
	void driveFieldRelative(float iX, float iY, float i0, double iSpeedModulation);
	void driveRobotRelative(wpi::math::ChassisVelocities iSpeeds);
	void modulesXFormation();
	void switchDriveType();

	wpi::math::Pose2d getPose();
	wpi::math::Translation2d getTranslationToHub();
	void resetPose(wpi::math::Pose2d iRobotPose);

	void resetIMU(wpi::units::degree_t iAngle);
	IMU* getIMU();

 private:
	wpi::math::Translation2d* mFrontLeftLocation;
	wpi::math::Translation2d* mFrontRightLocation;
	wpi::math::Translation2d* mBackLeftLocation;
	wpi::math::Translation2d* mBackRightLocation;

	wpi::nt::NetworkTableInstance inst = wpi::nt::NetworkTableInstance::GetDefault();
	std::shared_ptr<wpi::nt::NetworkTable> mNTDrivetrainTable = inst.GetTable("SmartDashboard/drivetrain");
	std::shared_ptr<wpi::nt::NetworkTable> mNTSwervePIDTable = inst.GetTable("SmartDashboard/swerve");

	wpi::nt::StructArrayPublisher<wpi::math::SwerveModuleVelocity> mCurrentModuleVelocitiesPublisher;
	wpi::nt::StructPublisher<wpi::math::ChassisVelocities> mCurrentChassisVelocityPublisher;
	wpi::nt::StructArrayPublisher<wpi::math::SwerveModuleVelocity> mDesiredModuleVelocitiesPublisher;
	wpi::nt::StructPublisher<wpi::math::ChassisVelocities> mDesiredChassisVelocityPublisher;
	wpi::nt::StructPublisher<wpi::math::Rotation2d> mRotation2dPublisher;
	wpi::nt::StructPublisher<wpi::math::Pose2d> mCurrentPose2dPublisher;
	wpi::nt::StructPublisher<wpi::math::Pose2d> mTargetPose2dPublisher;
	wpi::nt::StructPublisher<wpi::math::Translation2d> mTranslationToHubPublisher;
	wpi::nt::StructPublisher<wpi::math::Rotation2d> mRotationToHubPublisher;

	SwerveModule* mFrontLeftModule;
	SwerveModule* mFrontRightModule;
	SwerveModule* mBackLeftModule;
	SwerveModule* mBackRightModule;

	wpi::math::SwerveDriveKinematics<4>* mKinematics;
	wpi::math::Pose2d* mStartingRobotPose = new wpi::math::Pose2d{0_m, 0_m, 0_deg};
	wpi::math::SwerveDriveOdometry<4>* mOdometry;
	wpi::math::SwerveDrivePoseEstimator<4>* mPoseEstimator;

	wpi::Field2d* mField2d;

	bool mFieldRelative = true;

	wpi::util::array<double, 3>* visionMeasurementStdDevs;
	wpi::util::array<double, 3>* stateStdDevs;

	Limelight* mLimelight;

	// Declaring the IMU object
	IMU* mIMU;

	// These attributes are used to not create new variables every time a function is called
	std::optional<wpi::math::Pose2d> mLimelightEstimatedPose;
	wpi::math::ChassisVelocities mDesiredChassisVelocity;
	wpi::math::ChassisVelocities mCurrentChassisVelocity;
	wpi::math::Rotation2d mCurrentRotation2d;

	// The values are meant to be changed before being used
	wpi::util::array<wpi::math::SwerveModuleVelocity, 4> mDesiredSwerveStates = {wpi::math::SwerveModuleVelocity{0_mps, wpi::math::Rotation2d(0_rad)},
	                                                              wpi::math::SwerveModuleVelocity{0_mps, wpi::math::Rotation2d(0_rad)},
	                                                              wpi::math::SwerveModuleVelocity{0_mps, wpi::math::Rotation2d(0_rad)},
	                                                              wpi::math::SwerveModuleVelocity{0_mps, wpi::math::Rotation2d(0_rad)}};
};
