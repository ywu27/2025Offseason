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
    frc::PIDController{0.06, 0, 0}, // Change PIDs to be more accurate
    frc::PIDController{0.06, 0, 0}, // Change PIDs to be more accurate
    frc::ProfiledPIDController<units::radian>{
        0.4, 0, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            units::radians_per_second_t(5.0), // prev: 5.0
            units::radians_per_second_squared_t(100)}}}; // prev: 100

/**
 * Drives robot to the next state on trajectory
 * Odometry must be in meters
 */
void Trajectory::driveToState(PathPlannerTrajectoryState const &state)
{
    // Calculate new chassis speeds given robot position and next desired state in trajectory
    frc::ChassisSpeeds const correction = controller.Calculate(mDrive.getOdometryPose(), frc::Pose2d{state.pose.Translation(), state.heading}, state.linearVelocity, state.deltaRot);

    // Calculate x, y speeds from MPS
    double vy_feet = correction.vx.value() * 3.281;
    double vx_feet = correction.vy.value() * 3.281;

    // Clamp rot speed to 2.0 since that is the max rot we allow
    double rot = std::clamp(correction.omega.value(), -moduleMaxRot, moduleMaxRot);

    frc::SmartDashboard::PutNumber("autoVY", vy_feet);
    frc::SmartDashboard::PutNumber("autoVX", vx_feet);
    frc::SmartDashboard::PutNumber("autoRot", rot);

    mDrive.Drive(ChassisSpeeds{-vx_feet, vy_feet, rot}, mGyro.getBoundedAngleCCW(), true, true);
}

/**
 * Follows pathplanner trajectory
 */
void Trajectory::follow(std::string const &traj_dir_file_path, bool flipAlliance, bool intake, bool first, float startAngle)
{
    mDrive.enableModules();
    auto path = PathPlannerPath::fromPathFile(traj_dir_file_path);

    // switches path to red alliance (mirrors it)
    if (flipAlliance)
    {
        path = path->flipPath();
    }

    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), 0_rad, config);

    if (first)
    {
        auto const initialState = traj.getInitialState();
        auto const initialPose = initialState.pose.Translation();

        // set second param to initial holonomic rotation
        mDrive.resetOdometry(initialPose, units::angle::degree_t(startAngle));
    }

    frc::Timer trajTimer;
    trajTimer.Start();

    while ((mDrive.state == DriveState::Auto) && (trajTimer.Get() <= traj.getTotalTime()))
    {
        if (intake)
        {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        auto currentTime = trajTimer.Get();
        auto sample = traj.sample(currentTime);

        driveToState(sample);
        mDrive.updateOdometry();

        frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.getOdometryPose().Translation().X().value());
        frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.getOdometryPose().Translation().Y().value());

        using namespace std::chrono_literals;

        // refresh rate of holonomic drive controller's PID controllers (edit if needed)
        double delayStart = frc::Timer::GetFPGATimestamp().value();
        while (mDrive.state == DriveState::Auto && frc::Timer::GetFPGATimestamp().value() - delayStart < 0.02) {
        };
    }
    mDrive.stopModules();
}

// EDIT LATER 
/**
 * Calls sequences of follow functions for set paths
 */
void Trajectory::followPath(Trajectory::autos autoTrajectory, bool flipAlliance)
{
    switch (autoTrajectory)
    {
        case DO_NOTHING:
            break;
        case auto_1A:
            //follow("Test Movement", flipAlliance, false, true, 0.0);
            follow("1 to A", flipAlliance, false, true, 0.0);
            waitToShoot(2);
            follow("A to Top Coral Station", flipAlliance, false, false);
            waitToShoot(2);
            follow("Top Coral Station to A", flipAlliance, false, false);
            waitToShoot(2);
            follow("A to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_1B:
            follow("1 to B", flipAlliance, false, true, 0.0);
            waitToShoot(2);
            follow("B to Top Coral Station", flipAlliance, false, false);
            waitToShoot(2);
            follow("Top Coral Station to B", flipAlliance, false, false);
            waitToShoot(2);
            follow("B to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_1C:
            follow("1 to C", flipAlliance, false, true, 0.0);
            waitToShoot(2);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            waitToShoot(2);
            follow("Bottom Coral Station to C", flipAlliance, false, false);
            waitToShoot(2);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_1D:
            follow("1 to D", flipAlliance, false, true, 0.0);
            waitToShoot(2);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            waitToShoot(2);
            follow("Bottom Coral Station to D", flipAlliance, false, false);
            waitToShoot(2);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            break;
        // case auto_1E:
        //     follow("1 to E");
        //     follow("E to Bottom Coral Station");
        //     follow("Bottom Coral Station to E");
        //     follow("E to Bottom Coral Station");
        //     break;
        // case auto_1F:
        //     follow("1 to F");
        //     follow("F to Bottom Coral Station");
        //     follow("Bottom Coral Station to F");
        //     follow("F to Bottom Coral Station");
        //     break;
        // case auto_2A:
        //     follow("2 to A");
        //     follow("A to Top Coral Station");
        //     follow("Top Coral Station to A");
        //     follow("A to Top Coral Station");
        //     break;
        // case auto_2B:
        //     follow("2 to B");
        //     follow("B to Top Coral Station");
        //     follow("Top Coral Station to B");
        //     follow("B to Top Coral Station");
        //     break;
        // case auto_2C:
        //     follow("2 to C");
        //     follow("C to Bottom Coral Station");
        //     follow("Bottom Coral Station to C");
        //     follow("C to Bottom Coral Station");
        //     break;
        // case auto_2D:
        //     follow("2 to D");
        //     follow("D to Bottom Coral Station");
        //     follow("Bottom Coral Station to D");
        //     follow("D to Bottom Coral Stationn");
        //     break;
        // case auto_2E:
        //     follow("2 to E");
        //     follow("E to Bottom Coral Station");
        //     follow("Bottom Coral Station to E");
        //     follow("E to Bottom Coral Station");
        //     break;
        // case auto_2F:    
        //     follow("2 to F");
        //     follow("F to Bottom Coral Station");
        //     follow("Bottom Coral Station to F");
        //     follow("F to Bottom Coral Station");
        //     break;
        // case auto_3A:
        //     follow("3 to A");
        //     follow("A to Top Coral Station");
        //     follow("Top Coral Station to A");
        //     follow("A to Top Coral Station");
        //     break;
        // case auto_3B:
        //     follow("3 to B");
        //     follow("B to Top Coral Station");
        //     follow("Top Coral Station to B");
        //     follow("B to Top Coral Station");
        //     break;
        // case auto_3C:
        //     follow("3 to C");
        //     follow("C to Bottom Coral Station");
        //     follow("Bottom Coral Station to C");
        //     follow("C to Bottom Coral Station");
        //     break;
        // case auto_3D:
        //     follow("3 to D");
        //     follow("D to Bottom Coral Station");
        //     follow("Bottom Coral Station to D");
        //     follow("D to Bottom Coral Station");
        //     break;
        // case auto_3E:
        //     follow("3 to E");
        //     follow("E to Bottom Coral Station");
        //     follow("Bottom Coral Station to E");
        //     follow("E to Bottom Coral Station");
        //     break;
        // case auto_3F:
        //     follow("3 to F");
        //     follow("F to Bottom Coral Station");
        //     follow("Bottom Coral Station to F");
        //     follow("F to Bottom Coral Station");
        //     break;

    }
}

void Trajectory::waitToShoot(int delaySeconds)
{
    std::this_thread::sleep_for(std::chrono::seconds(delaySeconds));
}