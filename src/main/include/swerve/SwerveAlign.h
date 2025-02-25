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
    double currentX = 0;
    double currentY = 0;

public:

    bool isAligned(Limelight& limelight) {
        if (abs(limelight.getTX())<2 && abs(targetDistance-limelight.getDistanceToWall())<0.05) {
            return true;
        }
        return false;
    }

    ChassisSpeeds autoAlign(Limelight& limelight, SwerveHeadingController& headingController, double setpointDistance, double txSetpoint) { // distance in meters
        ChassisSpeeds speeds;
        double tx = limelight.getTX();
        double distanceToTag = limelight.getDistanceToWall();
        targetDistance = setpointDistance;
        if (!isAligned(limelight)) {
            double forwardSpeed = forwardPID.Calculate(distanceToTag, setpointDistance);
            double strafeSpeed = strafePID.Calculate(tx, txSetpoint);
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-forwardSpeed, -strafeSpeed, 0);
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