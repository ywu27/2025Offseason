#pragma once

#include "SwerveModule.h"
#include "Constants.h"
#include "util/ShuffleUI.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include "sensors/NavX.h"

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <thread>
#include "swerve/SwerveDriveKinematics.h"

// #define maxRot
// TODO: inherit thread helper
enum DriveState{
    Teleop,
    Auto,
    Disabled,
    Test
};
class SwerveDrive
{
private:
    SwerveModule mFrontLeft = SwerveModule(FLsteerID, FLdriveID, FL_CAN_ID);
    SwerveModule mFrontRight = SwerveModule(FRsteerID, FRdriveID, FR_CAN_ID);
    SwerveModule mBackLeft = SwerveModule(BLsteerID, BLdriveID, BL_CAN_ID);
    SwerveModule mBackRight = SwerveModule(BRsteerID, BRdriveID, BR_CAN_ID);

    // Threas for each Module
    std::thread modulePIDThread;
    float maxRot = moduleMaxRot;

    // TODO: Rename to Point2d
    std::array<Translation2d, 4> wheelPs = {Translation2d(trackWidthNumber, wheelBase), Translation2d(trackWidthNumber, -wheelBase), Translation2d(-trackWidthNumber, wheelBase), Translation2d(-trackWidthNumber, -wheelBase)};

    SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);
    NavX &mGyro; 

    // wpi lib class ver of kinemactics used to initialize odometry
    frc::SwerveDriveKinematics<4> frckinematics{ 
        frc::Translation2d{0.7239_m, 0.7239_m},
        frc::Translation2d{0.7239_m, -0.7239_m},
        frc::Translation2d{-0.7239_m, 0.7239_m},
        frc::Translation2d{-0.7239_m, -0.7239_m},
    };

    frc::SwerveDriveOdometry<4> m_odometry{
        frckinematics,
        mGyro.getRotation2d(), 
        // might need to edit order of motors (double check)
        {
            mBackLeft.getModulePosition(), 
            mFrontLeft.getModulePosition(), 
            mFrontRight.getModulePosition(), 
            mBackRight.getModulePosition() 
        },
        frc::Pose2d{0_m, 0_m, 0_deg}
    };

    
    
    // Module Level functions
    void runModules(); // Private - do not call outside of init

public:

    SwerveDrive(NavX &mGyroInput) : mGyro(mGyroInput) {
    }

    DriveState state = DriveState::Teleop;
    // TODO overload - pass Point2d + rotation, it figures it out
    // void Drive(Translation2d translation, Rotation2d rotation);
    void Drive(ChassisSpeeds desiredSpeeds, Rotation2d fieldRelativeGyro, bool useFieldOriented, bool cleanAccum = false);
    void initModules();
    void enableModules();
    bool stopModules();
    void orientModules(double FL, double FR, double BL, double BR);
    void autoMove(double angleRadians, double distanceFeet);
    void resetOdometry(frc::Translation2d trans, frc::Rotation2d angle);
    frc::Pose2d getOdometryPose();
    void updateOdometry();
    void displayDriveTelemetry();
    void setDriveCurrentLimit(int limit);
    void zeroAccumulation();

};