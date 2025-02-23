#pragma once

#include "SwerveDrive.h"
#include "Constants.h"
#include "sensors/Limelight.h"
#include "geometry/Translation2d.h"

#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/GoalEndState.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"

#include "units/velocity.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

#include <chrono>
#include <frc/Timer.h>

using namespace pathplanner;

class Trajectory
{
private:
    Pose3d startPose = Pose3d();
    bool receivedPose;
    bool isRed = false;
    SwerveDrive &mDrive;
    NavX &mGyro;
    RobotConfig &config;
    enum autos {
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
public:
    Trajectory(SwerveDrive &mDriveInput, NavX &mGyroInput, RobotConfig &configInput) : mDrive(mDriveInput), mGyro(mGyroInput), config(configInput){}; // Check patherplanner::RobotConfig documentation 

    void driveToState(PathPlannerTrajectoryState const &state);

    void follow(std::string const &traj_dir_file_path);

    void followPath(autos autopath);

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);

    void waitToShoot(int delaySeconds);
};