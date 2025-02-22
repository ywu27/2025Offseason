#include "Trajectory.h"
#include "SwerveDrive.h"
#include "SwerveModule.h"
#include "Robot.h"

#include <pathplanner/lib/trajectory/PathPlannerTrajectory.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/GoalEndState.h>

#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>

// controller used to track trajectories + correct minor disturbances
static frc::HolonomicDriveController controller{
    frc::PIDController{0.1, 0, 0},
    frc::PIDController{0.1, 0, 0},
    frc::ProfiledPIDController<units::radian>{
        steerP, 0, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            units::radians_per_second_t(189.2), // prev: 5.0
            units::radians_per_second_squared_t(2665.993 * (25.8 / 7.6))}}}; // prev: 100

/**
 * Drives robot to the next state on trajectory
 * Odometry must be in meters
 */
void Trajectory::driveToState(PathPlannerTrajectoryState const &state)
{   
    // Calculate new chassis speeds given robot position and next desired state in trajectory
    frc::ChassisSpeeds const correction = controller.Calculate(mDrive.getOdometryPose(), state.pose, state.linearVelocity, state.deltaRot);

    // Calculate x, y speeds from MPS
    double vx_feet = correction.vx.value() * 3.281;
    double vy_feet = correction.vy.value() * 3.281;

    // Clamp rot speed to 2.0 since that is the max rot we allow
    double rot = std::clamp(correction.omega.value(), -moduleMaxRot, moduleMaxRot);

    frc::SmartDashboard::PutNumber("autoVY", vy_feet);
    frc::SmartDashboard::PutNumber("autoVX", vx_feet);
    frc::SmartDashboard::PutNumber("autoRot", rot);

    mDrive.Drive(ChassisSpeeds{-vy_feet, vx_feet, rot}, mGyro.getBoundedAngleCCW(), true, true);
}

/**
 * Follows pathplanner trajectory
 */
void Trajectory::follow(std::string const &traj_dir_file_path)
{
    auto path = PathPlannerPath::fromPathFile(traj_dir_file_path);
    PathPlannerTrajectory traj = PathPlannerTrajectory(path,  frc::ChassisSpeeds(), frc::Rotation2d(0_rad), config);

    auto const initialState = traj.getInitialState();
    auto const initialPose = initialState.pose;
    mDrive.resetOdometry(initialPose.Translation(), initialState.heading);

    frc::Timer trajTimer;
    trajTimer.Start();

    while ((mDrive.state == DriveState::Auto) && (trajTimer.Get() <= traj.getTotalTime()))
    {
        auto currentTime = trajTimer.Get();
        auto sample = traj.sample(currentTime);

        driveToState(sample);
        mDrive.updateOdometry();

        frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.getOdometryPose().Translation().X().value());
        frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.getOdometryPose().Translation().Y().value());

        using namespace std::chrono_literals;

        // refresh rate of holonomic drive controller's PID controllers (edit if needed)
        std::this_thread::sleep_for(20ms);
    }
    mDrive.stopModules();
}

/**
 * Calls sequences of follow functions for set paths
 */
void Trajectory::followPath(int numPath)
{
    switch (numPath)
    {
    // do nothing
    case 0: 
        break; 

    // straight
    case 1:
        follow("Straight");
        break;

    // straight, shoot
    case 2:
        follow("Straight");
        //  CALL SUPERSTRUCTURE SHOOT SPEAKER FOR SHOOTER AUTO
        break;

    // straight, shoot, straight
    case 3:
        follow("Straight");
        break;

    // curve
    case 4:
        follow("Curve");
        break;

    default:
        break;
    }
}