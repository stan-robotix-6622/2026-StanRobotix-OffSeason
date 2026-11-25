// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubDrivetrain.hpp"

#include <wpi/system/DataLogManager.hpp>
#include "wpi/driverstation/MatchState.hpp"
#include "wpi/driverstation/RobotState.hpp"
#include "wpi/driverstation/Alliance.hpp"
#include "wpi/driverstation/MatchType.hpp"
#include <wpi/framework/RobotBase.hpp>
#include <wpi/system/Timer.hpp>
// #include <wpi/smartdashboard/SmartDashboard.hpp>
// #include <pathplanner/lib/auto/AutoBuilder.h>
// #include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
// #include <pathplanner/lib/util/PathPlannerLogging.h>

#include <memory>
#include <numbers>
#include <string>
#include <vector>

#include "Constants.hpp"

SubDrivetrain::SubDrivetrain()
{
	mFrontLeftLocation = new wpi::math::Translation2d{ChassisConstants::kFrontLeftTranslation};
	mFrontRightLocation = new wpi::math::Translation2d{ChassisConstants::kFrontRightTranslation};
	mBackLeftLocation = new wpi::math::Translation2d{ChassisConstants::kBackLeftTranslation};
	mBackRightLocation = new wpi::math::Translation2d{ChassisConstants::kBackRightTranslation};

	mFrontLeftModule = new SwerveModule{CANid::kFrontLeftMotorID, CANid::kFrontLeftMotor550ID, false};
	mFrontRightModule = new SwerveModule{CANid::kFrontRightMotorID, CANid::kFrontRightMotor550ID, false};
	mBackLeftModule = new SwerveModule{CANid::kBackLeftMotorID, CANid::kBackLeftMotor550ID, true};
	mBackRightModule = new SwerveModule{CANid::kBackRightMotorID, CANid::kBackRightMotor550ID, true};

	mCurrentModuleVelocitiesPublisher = mNTDrivetrainTable->GetStructArrayTopic<wpi::math::SwerveModuleVelocity>("Current SwerveModuleVelocities").Publish();
	mCurrentChassisVelocityPublisher = mNTDrivetrainTable->GetStructTopic<wpi::math::ChassisVelocities>("Current wpi::math::ChassisVelocities").Publish();
	mDesiredModuleVelocitiesPublisher = mNTDrivetrainTable->GetStructArrayTopic<wpi::math::SwerveModuleVelocity>("Desired SwerveModuleVelocities").Publish();
	mDesiredChassisVelocityPublisher = mNTDrivetrainTable->GetStructTopic<wpi::math::ChassisVelocities>("Desired wpi::math::ChassisVelocities").Publish();
	mRotation2dPublisher = mNTDrivetrainTable->GetStructTopic<wpi::math::Rotation2d>("Current wpi::math::Rotation2d").Publish();
	mCurrentPose2dPublisher = mNTDrivetrainTable->GetStructTopic<wpi::math::Pose2d>("Current wpi::math::Pose2d").Publish();
	mTargetPose2dPublisher = mNTDrivetrainTable->GetStructTopic<wpi::math::Pose2d>("Target wpi::math::Pose2d").Publish();
	mTranslationToHubPublisher = mNTDrivetrainTable->GetStructTopic<wpi::math::Translation2d>("Translation to Hub").Publish();
	mRotationToHubPublisher = mNTDrivetrainTable->GetStructTopic<wpi::math::Rotation2d>("Rotation to Hub").Publish();

	mLimelight = new Limelight{LimelightConstants::kName};
	mLimelight->setCameraPosition(
			LimelightConstants::kForward,
			LimelightConstants::kRight,
			LimelightConstants::kUp,
			LimelightConstants::kRoll,
			LimelightConstants::kPitch,
			LimelightConstants::kYaw);

	mIMU = new IMU{};
	mIMU->reset();
	// wpi::SmartDashboard::PutData("drivetrain/IMU", mIMU);

	mKinematics = new wpi::math::SwerveDriveKinematics<4>{*mFrontLeftLocation, *mFrontRightLocation, *mBackLeftLocation, *mBackRightLocation};
	mPoseEstimator = new wpi::math::SwerveDrivePoseEstimator<4>{*mKinematics, mIMU->getRotation2d(), getSwerveModulePositions(), *mStartingRobotPose};

	visionMeasurementStdDevs = new wpi::util::array<double, 3>{LimelightConstants::kPoseEstimatorStandardDeviationX,
	                                                     LimelightConstants::kPoseEstimatorStandardDeviationY,
	                                                     LimelightConstants::kPoseEstimatorStandardDeviationYaw};
	mPoseEstimator->SetVisionMeasurementStdDevs(*visionMeasurementStdDevs);

	mField2d = new wpi::Field2d{};
	// wpi::SmartDashboard::PutData("drivetrain/Field2d", mField2d);

	ConfigurePathplanner();
}

void SubDrivetrain::Periodic()
{
	refreshSwerveModules();
	mCurrentRotation2d = mIMU->getRotation2d();
	mPoseEstimator->Update(mCurrentRotation2d, getSwerveModulePositions());
	mField2d->SetRobotPose(getPose());

	mLimelightEstimatedPose = mLimelight->getPoseEstimation(getPose(), mIMU->getYawRate(), mFieldRelative);
	if (mLimelightEstimatedPose) {
		mPoseEstimator->AddVisionMeasurement(mLimelightEstimatedPose.value(), wpi::Timer::GetMonotonicTimestamp());
	}

	// Publication de valeurs sur le NetworkTables
	mCurrentChassisVelocityPublisher.Set(getRobotRelativeSpeeds());
	mCurrentModuleVelocitiesPublisher.Set(getSwerveModuleVelocities());
	mRotation2dPublisher.Set(mCurrentRotation2d.Degrees());
	mCurrentPose2dPublisher.Set(mPoseEstimator->GetEstimatedPosition());
}

void SubDrivetrain::switchDriveType()
{
	mFieldRelative = !mFieldRelative;
}

void SubDrivetrain::setSwerveModuleVelocities(wpi::util::array<wpi::math::SwerveModuleVelocity, 4> iStates)
{
	mFrontLeftModule->setDesiredState(iStates[0]);
	mFrontRightModule->setDesiredState(iStates[1]);
	mBackLeftModule->setDesiredState(iStates[2]);
	mBackRightModule->setDesiredState(iStates[3]);

	if (wpi::RobotBase::IsSimulation()) {
		mIMU->advanceSimulation(mKinematics->ToChassisVelocities(iStates).omega);
	}
}

void SubDrivetrain::ConfigurePathplanner()
{
	wpi::DataLogManager::Log("Started PathPlanner Configuration");

	// Load the RobotConfig from the GUI settings. You should probably
	// store this in your Constants file
	// pathplanner::RobotConfig PathPlannerConfig = pathplanner::RobotConfig::fromGUISettings();

	// pathplanner::AutoBuilder::configure(
	// 		[this]() { return getPose(); },                                                                                                               // Robot pose supplier
	// 		[this](wpi::math::Pose2d pose) { resetPose(pose); },                                                                                                // Method to reset odometry (will be called if your auto has a starting pose)
	// 		[this]() { return getRobotRelativeSpeeds(); },                                                                                                // wpi::math::ChassisVelocities supplier. MUST BE ROBOT RELATIVE
	// 		[this](auto speeds, auto feedforwards) { driveRobotRelative(speeds); },                                                                       // Method that will drive the robot given ROBOT RELATIVE wpi::math::ChassisVelocities. Also optionally outputs individual module feedforwards
	// 		std::make_shared<pathplanner::PPHolonomicDriveController>(                                                                                    // PPHolonomicController is the built in path following controller for holonomic drive trains
	// 				pathplanner::PIDConstants(PathPlannerConstants::kPTranslation, PathPlannerConstants::kITranslation, PathPlannerConstants::kDTranslation), // Translation PID constants
	// 				pathplanner::PIDConstants(PathPlannerConstants::kPRotation, PathPlannerConstants::kIRotation, PathPlannerConstants::kDRotation)           // Rotation PID constants
	// 				),
	// 		PathPlannerConfig, // The robot configuration
	// 		[]() {
	// 			// Boolean supplier that controls when the path will be mirrored for the red alliance
	// 	    // This will flip the path being followed to the red side of the field.
	// 	    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

	// 			std::optional<wpi::DriverStation::Alliance> alliance = wpi::MatchState::GetAlliance();
	// 			if (alliance) {
	// 				return alliance.value() == wpi::DriverStation::Alliance::kRed;
	// 			}
	// 			return false;
	// 		},
	// 		this // Reference to this subsystem to set requirements
	// );
	wpi::DataLogManager::Log("Finished Autobuilder Configuration");

	// // Logging callback for the active path, this is sent as a vector of poses
	// pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<wpi::math::Pose2d> poses) {
	// 	// Do whatever you want with the poses here
	// 	mField2d->GetObject("path")->SetPoses(poses);
	// });
	wpi::DataLogManager::Log("Finished Pathplanner Configuration");
}

void SubDrivetrain::refreshSwerveModules()
{
	mFrontLeftModule->refreshModule();
	mFrontRightModule->refreshModule();
	mBackLeftModule->refreshModule();
	mBackRightModule->refreshModule();
}

wpi::util::array<wpi::math::SwerveModuleVelocity, 4> SubDrivetrain::getSwerveModuleVelocities()
{
	return wpi::util::array<wpi::math::SwerveModuleVelocity, 4>{mFrontLeftModule->getModuleVelocity(),
	                                             mFrontRightModule->getModuleVelocity(),
	                                             mBackLeftModule->getModuleVelocity(),
	                                             mBackRightModule->getModuleVelocity()};
}

wpi::util::array<wpi::math::SwerveModulePosition, 4> SubDrivetrain::getSwerveModulePositions()
{
	return wpi::util::array<wpi::math::SwerveModulePosition, 4>{mFrontLeftModule->getModulePosition(),
	                                                mFrontRightModule->getModulePosition(),
	                                                mBackLeftModule->getModulePosition(),
	                                                mBackRightModule->getModulePosition()};
}

// wpi::cmd::CommandPtr SubDrivetrain::getDriveCommand(std::function<double()> iXSupplier,
// 																								std::function<double()> iYSupplier,
// 																								std::function<double()> i0Supplier,
// 																	 							std::function<double()> iSpeedModulationSupplier,
// 																								bool iFieldRelative)
// {
// 	return this->Run([this, iXSupplier, iYSupplier, i0Supplier, iSpeedModulationSupplier, iFieldRelative] {
// 	if (iFieldRelative) {
// 		if (wpi::MatchState::GetAlliance().value() == wpi::Alliance::BLUE) {
// 			mDesiredChassisVelocity = wpi::math::ChassisVelocities{iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredSpeed * iXSupplier(),
// 																					iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredSpeed * iYSupplier(),
// 																					iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredAngularSpeed * i0Supplier()};
// 		}
// 		else {
// 			mDesiredChassisVelocity = wpi::math::ChassisVelocities{iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredSpeed * -iXSupplier(),
// 																					iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredSpeed * -iYSupplier(),
// 																					iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredAngularSpeed * i0Supplier()};
// 		}
// 	}
// 	else {
// 		mDesiredChassisVelocity = wpi::math::ChassisVelocities{iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredSpeed * iXSupplier(),
// 																				iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredSpeed * iYSupplier(),
// 																				iSpeedModulationSupplier() * DrivetrainConstants::kMaxDesiredAngularSpeed * i0Supplier()};
// 	}
// 	mDesiredChassisVelocity.ToFieldRelative(getPose().Rotation());

// 	mDesiredSwerveStates = mKinematics->ToSwerveModuleVelocities(mDesiredChassisVelocity); // The array has in order: fl, fr, bl, br
// 	mDesiredSwerveStates = mKinematics->DesaturateWheelVelocities(mDesiredSwerveStates, DrivetrainConstants::kAttainableSpeed);

// 	mDesiredChassisVelocityPublisher.Set(mDesiredChassisVelocity);
// 	mDesiredModuleVelocitiesPublisher.Set(mDesiredSwerveStates);

// 	setSwerveModuleVelocities(mDesiredSwerveStates);}).WithName("DriveCommand");
// }

void SubDrivetrain::driveFieldRelative(float iX, float iY, float i0, double iSpeedModulation)
{
	if (mFieldRelative) {
		if (wpi::MatchState::GetAlliance().value() == wpi::Alliance::BLUE) {
			mDesiredChassisVelocity = wpi::math::ChassisVelocities{iSpeedModulation * DrivetrainConstants::kMaxDesiredSpeed * iX,
			                                                                    iSpeedModulation * DrivetrainConstants::kMaxDesiredSpeed * iY,
			                                                                    iSpeedModulation * DrivetrainConstants::kMaxDesiredAngularSpeed * i0};
		}
		else {
			mDesiredChassisVelocity = wpi::math::ChassisVelocities{iSpeedModulation * DrivetrainConstants::kMaxDesiredSpeed * -iX,
			                                                                    iSpeedModulation * DrivetrainConstants::kMaxDesiredSpeed * -iY,
			                                                                    iSpeedModulation * DrivetrainConstants::kMaxDesiredAngularSpeed * i0};
		}
	}
	else {
		mDesiredChassisVelocity = wpi::math::ChassisVelocities{iSpeedModulation * DrivetrainConstants::kMaxDesiredSpeed * iX,
		                                                                    iSpeedModulation * DrivetrainConstants::kMaxDesiredSpeed * iY,
		                                                                    iSpeedModulation * DrivetrainConstants::kMaxDesiredAngularSpeed * i0};
	}
	mDesiredChassisVelocity.ToFieldRelative(getPose().Rotation());

	mDesiredSwerveStates = mKinematics->ToSwerveModuleVelocities(mDesiredChassisVelocity); // The array has in order: fl, fr, bl, br
	mDesiredSwerveStates = mKinematics->DesaturateWheelVelocities(mDesiredSwerveStates, DrivetrainConstants::kAttainableSpeed);

	mDesiredChassisVelocityPublisher.Set(mDesiredChassisVelocity);
	mDesiredModuleVelocitiesPublisher.Set(mDesiredSwerveStates);

	setSwerveModuleVelocities(mDesiredSwerveStates);
}

void SubDrivetrain::mesureSwerveFeedforward(wpi::units::volt_t iDrivingVoltage, wpi::util::array<wpi::math::Rotation2d, 4> iDesiredHeadings)
{
	mFrontLeftModule->setDrivingVoltage(iDrivingVoltage);
	mFrontRightModule->setDrivingVoltage(iDrivingVoltage);
	mBackLeftModule->setDrivingVoltage(iDrivingVoltage);
	mBackRightModule->setDrivingVoltage(iDrivingVoltage);

	mFrontLeftModule->setDesiredHeading(iDesiredHeadings[0]);
	mFrontRightModule->setDesiredHeading(iDesiredHeadings[1]);
	mBackLeftModule->setDesiredHeading(iDesiredHeadings[2]);
	mBackRightModule->setDesiredHeading(iDesiredHeadings[3]);

	// wpi::SmartDashboard::PutNumber("drivetrain/Driving Voltage", iDrivingVoltage.value());
	// wpi::SmartDashboard::PutNumber("drivetrain/Driving Velocity", mFrontLeftModule->getModuleVelocity().speed.value());
}

wpi::math::Pose2d SubDrivetrain::getPose()
{
	return mPoseEstimator->GetEstimatedPosition();
}

void SubDrivetrain::resetPose(wpi::math::Pose2d iRobotPose)
{
	mPoseEstimator->ResetPosition(mIMU->getRotation2d(), getSwerveModulePositions(), iRobotPose);
}

void SubDrivetrain::resetIMU(wpi::units::degree_t iAngle)
{
	mPoseEstimator->ResetRotation(iAngle);
	mIMU->setAngleYaw(iAngle);
}

IMU* SubDrivetrain::getIMU()
{
	return mIMU;
}

wpi::math::ChassisVelocities SubDrivetrain::getRobotRelativeSpeeds()
{
	mCurrentChassisVelocity = mKinematics->ToChassisVelocities(getSwerveModuleVelocities());
	return mCurrentChassisVelocity;
}

wpi::math::ChassisVelocities SubDrivetrain::getFieldRelativeSpeeds()
{
	mCurrentChassisVelocity = mKinematics->ToChassisVelocities(getSwerveModuleVelocities());
	return mCurrentChassisVelocity.ToFieldRelative(getPose().Rotation());
}

void SubDrivetrain::driveRobotRelative(wpi::math::ChassisVelocities iDesiredChassisVelocity)
{
	mDesiredSwerveStates = mKinematics->ToSwerveModuleVelocities(iDesiredChassisVelocity); // The array has in order: fl, fr, bl, br

	setSwerveModuleVelocities(mDesiredSwerveStates);
}

void SubDrivetrain::modulesXFormation()
{
	mFrontLeftModule->setDesiredHeading(45_deg);
	mFrontRightModule->setDesiredHeading(135_deg);
	mBackLeftModule->setDesiredHeading(135_deg);
	mBackRightModule->setDesiredHeading(45_deg);
}

// wpi::cmd::CommandPtr SubDrivetrain::getFollowPathCommand(std::string iPathName)
// {
// 	// // wPath is of type std::shared_ptr<pathplanner::PathPlannerPath>
// 	// auto wPath = pathplanner::PathPlannerPath::fromPathFile(iPathName);

// 	// return pathplanner::AutoBuilder::followPath(wPath);
// 	return this->Idle().WithName("PathFollowingCommand");
// }

// void SubDrivetrain::InitSendable(wpi::util::SendableBuilder& builder)
// {
// 	builder.SetSmartDashboardType("SwerveDrive");

// 	builder.AddDoubleProperty("Front Left Angle", [this] { return mFrontLeftModule->getModuleVelocity().angle.Radians().value(); }, nullptr);
// 	builder.AddDoubleProperty("Front Left Velocity", [this] { return mFrontLeftModule->getModuleVelocity().speed.value(); }, nullptr);

// 	builder.AddDoubleProperty("Front Right Angle", [this] { return mFrontRightModule->getModuleVelocity().angle.Radians().value(); }, nullptr);
// 	builder.AddDoubleProperty("Front Right Velocity", [this] { return mFrontRightModule->getModuleVelocity().speed.value(); }, nullptr);

// 	builder.AddDoubleProperty("Back Left Angle", [this] { return mBackLeftModule->getModuleVelocity().angle.Radians().value(); }, nullptr);
// 	builder.AddDoubleProperty("Back Left Velocity", [this] { return mBackLeftModule->getModuleVelocity().speed.value(); }, nullptr);

// 	builder.AddDoubleProperty("Back Right Angle", [this] { return mBackRightModule->getModuleVelocity().angle.Radians().value(); }, nullptr);
// 	builder.AddDoubleProperty("Back Right Velocity", [this] { return mBackRightModule->getModuleVelocity().speed.value(); }, nullptr);

// 	builder.AddDoubleProperty("Robot Angle", [this] { return getPose().Rotation().Radians().value(); }, nullptr);
// }
