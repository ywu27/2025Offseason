#pragma once

#include "sensors/Limelight.h"
#include "ChassisSpeeds.h"
#include "SwerveHeadingController.h"
#include "SwerveDrive.h"

class SwerveAlign {
private:
    frc::PIDController forwardPID{7, 0, 0.1};
    frc::PIDController strafePID{0.5, 0, 0.1};
    
    double forwardSpeed = 0;
    double strafeSpeed = 0;
    double targetDistance = 0;
    double targetOffset = 0;
    double currentX = 0;
    double currentY = 0;

public:

    bool isAligned(Limelight& limelight) {
        if (abs(limelight.getTargetPoseRobotSpace().x-targetOffset)<0.05 && abs(targetDistance-limelight.getDistanceToWall())<0.05) {
            return true;
        }
        return false;
    }

    ChassisSpeeds autoAlign(Limelight& limelight, double setpointDistance, double offsetSetpoint) { // distance in meters
        ChassisSpeeds speeds;
        double offset = limelight.getTargetPoseRobotSpace().x;
        double distanceToTag = limelight.getDistanceToWall();
        targetDistance = setpointDistance;
        targetOffset = offsetSetpoint;
        if (!isAligned(limelight)) {
            double forwardSpeed = forwardPID.Calculate(distanceToTag, setpointDistance);
            double strafeSpeed = strafePID.Calculate(offset, offsetSetpoint);
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-strafeSpeed, -forwardSpeed, 0); //CHECK THIS
        }
        else {
            speeds = ChassisSpeeds(0, 0, 0);
        }
        return speeds;
    }

    ChassisSpeeds driveToSetpoint(double setpointX, double setpointY, SwerveDrive& drive) {
        ChassisSpeeds speeds;
        if (abs(setpointX-currentX)>0.2 && abs(setpointY-currentY)>0.2) {

            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-strafeSpeed, -forwardSpeed, 0);
        }
        else {
            speeds = ChassisSpeeds(0, 0, 0);
        }
    }
};