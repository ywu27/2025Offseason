#pragma once

#include "SwerveDrive.h"
#include "Constants.h"
#include "sensors/Limelight.h"
#include "geometry/Translation2d.h"
#include "swerve/SwerveAlign.h"
#include "swerve/SwerveHeadingController.h"
#include "Superstructure.h"

#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/GoalEndState.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"

#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "units/velocity.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

#include <chrono>
#include <frc/Timer.h>
#include "sensors/Pigeon.h"
#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>

#include <sensors/PhotonVision.h>

using namespace pathplanner;

class Trajectory
{
private:
    frc::Timer delayTimer;
    SwerveDrive &mDrive;
    Superstructure &mSuperstructure;
    SwerveAlign &mAlign;
    Pigeon &pigeon;
    Limelight& mLimelight1;
    Limelight& mLimelight2;
    RobotConfig &config;
    PhotonVision &camera;
    SwerveHeadingController mHeadingController;

public:
    Pose3d startPose = Pose3d();
    bool receivedPose;
    bool isRed = false;

    frc::Timer alignTimer;

    enum autos {
        DO_NOTHING,
        MOVE_STRAIGHT,
        auto_1A,  
        auto_1B,  
        auto_1C,  
        auto_1D,  
        auto_1E,  
        auto_1F,  
        auto_2A,  
        auto_2B,  
        auto_2C,  
        auto_2D,  
        auto_2E,  
        auto_2F,  
        auto_3A,  
        auto_3B,  
        auto_3C,  
        auto_3D,  
        auto_3E,  
        auto_3F 
    };

    Trajectory(SwerveDrive &mDriveInput, Superstructure &mSuperstructure, SwerveHeadingController &mHeadingController, Limelight& limelight1, Limelight& limelight2, PhotonVision &cameraInput, SwerveAlign &align, Pigeon &pigeonInput, RobotConfig &configInput) : mDrive(mDriveInput), 
                                                                                                                mSuperstructure(mSuperstructure),
                                                                                                                mHeadingController(mHeadingController),
                                                                                                                mLimelight1(limelight1),
                                                                                                                mLimelight2(limelight2),
                                                                                                                mAlign(align),
                                                                                                                pigeon(pigeonInput),
                                                                                                                config(configInput),
                                                                                                                camera(cameraInput) {};

    void driveToState(PathPlannerTrajectoryState const &state);

    void follow(std::string const &traj_dir_file_path, bool flipAlliance, bool intake, bool first, float startAngle = 0.0);

    void followPath(Trajectory::autos autoTrajectory, bool flipAlliance);

    void waitToScore(int delaySeconds);

    void driveError(); 

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);

    void followTeleop(std::shared_ptr<pathplanner::PathPlannerPath> path, bool flipAlliance);
};