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
void Trajectory::followPath(Trajectory::autos autopath)
{
    switch (autopath)
    {
    // do nothing
        case auto_1A:
            follow("1 to A");
            follow("A to Top Coral Station");
            follow("Top Coral Station to A");
            follow("A to Top Coral Station");
            break;
        case auto_1B:
            follow("1 to B");
            follow("B to Top Coral Station");
            follow("Top Coral Station to B");
            follow("B to Top Coral Station");
            break;
        case auto_1C:
            follow("1 to C");
            follow("C to Bottom Coral Station");
            follow("Bottom Coral Station to C");
            follow("C to Bottom Coral Station");
            break;
        case auto_1D:
            follow("1 to D");
            follow("D to Bottom Coral Station");
            follow("Bottom Coral Station to D");
            follow("D to Bottom Coral Station");
            break;
        case auto_1E:
            follow("1 to E");
            follow("E to Bottom Coral Station");
            follow("Bottom Coral Station to E");
            follow("E to Bottom Coral Station");
            break;
        case auto_1F:
            follow("1 to F");
            follow("F to Bottom Coral Station");
            follow("Bottom Coral Station to F");
            follow("F to Bottom Coral Station");
            break;
        case auto_2A:
            follow("2 to A");
            follow("A to Top Coral Station");
            follow("Top Coral Station to A");
            follow("A to Top Coral Station");
            break;
        case auto_2B:
            follow("2 to B");
            follow("B to Top Coral Station");
            follow("Top Coral Station to B");
            follow("B to Top Coral Station");
            break;
        case auto_2C:
            follow("2 to C");
            follow("C to Bottom Coral Station");
            follow("Bottom Coral Station to C");
            follow("C to Bottom Coral Station");
            break;
        case auto_2D:
            follow("2 to D");
            follow("D to Bottom Coral Station");
            follow("Bottom Coral Station to D");
            follow("D to Bottom Coral Stationn");
            break;
        case auto_2E:
            follow("2 to E");
            follow("E to Bottom Coral Station");
            follow("Bottom Coral Station to E");
            follow("E to Bottom Coral Station");
            break;
        case auto_2F:    
            follow("2 to F");
            follow("F to Bottom Coral Station");
            follow("Bottom Coral Station to F");
            follow("F to Bottom Coral Station");
            break;
        case auto_3A:
            follow("3 to A");
            follow("A to Top Coral Station");
            follow("Top Coral Station to A");
            follow("A to Top Coral Station");
            break;
        case auto_3B:
            follow("3 to B");
            follow("B to Top Coral Station");
            follow("Top Coral Station to B");
            follow("B to Top Coral Station");
            break;
        case auto_3C:
            follow("3 to C");
            follow("C to Bottom Coral Station");
            follow("Bottom Coral Station to C");
            follow("C to Bottom Coral Station");
            break;
        case auto_3D:
            follow("3 to D");
            follow("D to Bottom Coral Station");
            follow("Bottom Coral Station to D");
            follow("D to Bottom Coral Station");
            break;
        case auto_3E:
            follow("3 to E");
            follow("E to Bottom Coral Station");
            follow("Bottom Coral Station to E");
            follow("E to Bottom Coral Station");
            break;
        case auto_3F:
            follow("3 to F");
            follow("F to Bottom Coral Station");
            follow("Bottom Coral Station to F");
            follow("F to Bottom Coral Station");
            break;

    }

    
}

void Trajectory::waitToShoot(int delaySeconds)
{
    //will finish when elevator is assembled and ready...closer to comp
}