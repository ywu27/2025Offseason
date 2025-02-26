#pragma once

#include "SwerveDrive.h"
#include "Constants.h"
#include "sensors/Limelight.h"
#include "geometry/Translation2d.h"

#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/GoalEndState.h>
#include <pathplanner/lib/config/RobotConfig.h>

#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Pose2d.h"

#include "units/velocity.h"
#include "units/angle.h"
#include "units/angular_acceleration.h"
#include "units/angular_velocity.h"

#include <chrono>
#include <frc/Timer.h>
#include "sensors/NavX.h"
#include <frc/DriverStation.h>

using namespace pathplanner;

class Trajectory
{
private:
    
    SwerveDrive &mDrive;
    NavX &mGyro;
    Limelight &mLimelight; 
    RobotConfig &config;

public:
    Pose3d startPose = Pose3d();
    bool receivedPose;
    bool isRed = false;

    enum autos {
        DO_NOTHING,
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

    Trajectory(SwerveDrive &mDriveInput, NavX &mGyroInput, Limelight &mLimelightInput, RobotConfig &configInput) : mDrive(mDriveInput), 
                                                                                                                mGyro(mGyroInput),
                                                                                                                mLimelight(mLimelightInput),
                                                                                                                config(configInput) {};

    void driveToState(PathPlannerTrajectoryState const &state);

    void follow(std::string const &traj_dir_file_path, bool flipAlliance, bool intake, bool first, float startAngle = 0.0);

    void followPath(Trajectory::autos autoTrajectory, bool flipAlliance);

    void waitToShoot(int delaySeconds);

    void driveError(); 

    void testHolonomic(frc::Pose2d const &target_pose,
                       units::velocity::meters_per_second_t const &velocity,
                       frc::Rotation2d const &target_rot);
};