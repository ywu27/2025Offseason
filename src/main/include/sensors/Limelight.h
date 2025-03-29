#pragma once

#include <vector>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include "LimelightHelpers.h"
#include "geometry/Pose3d.h"
#include "Constants.h"
#include <frc/DriverStation.h>

class Limelight {

private:
    std::string limelightName;
    double limelightMountAngle = 20;
    double limelightHeight = 12;
    
    struct TagInfo {
        double height;
        int angleSetpoint;
    };
    
    std::map<int, TagInfo> tagData = {
        {1, {53.25, 126}}, {2, {53.25, 234}}, {3, {45.875, 270}}, {4, {69.0, 0}}, {5, {69.0, 0}},
        {10, {6.875, 180}}, {11, {6.875, 240}}, {6, {6.875, 300}}, {7, {6.875, 0}}, {8, {6.875, 60}}, {9, {6.875, 120}},
        {12, {53.25, 234}}, {13, {53.25, 126}}, {14, {69.0, 0}}, {15, {69.0, 0}}, {16, {45.875, 270}},
        {17, {6.875, 120}}, {18, {6.875, 60}}, {19, {6.875, 0}}, {20, {6.875, 300}}, {21, {6.875, 240}}, {22, {6.875, 180}}
    };

public:
    enum Alliance{RED, BLUE};
    enum TagType {REEF, CORALSTATION, PROCESSOR, BARGE};
    
    Alliance alliance;
    TagType tagType;

    Limelight(std::string name, frc::DriverStation::Alliance a) { // TEST THIS
        limelightName = name;
        
        if (a == frc::DriverStation::Alliance::kRed) {
            alliance = RED;
        }
        else {
            alliance = BLUE;
        }
    }

    bool isTargetDetected() {
        double tv = nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->GetNumber("tv", 0.0);
        if (tv == 1) {
            return true;
        } else {
            return false;
        }
    }

    bool isTargetDetected2() {
        if (getTX()==0 && getTY()==0) {
            return false;
        }
        return true;
    }

    void setPipelineIndex(int index) {
        LimelightHelpers::setPipelineIndex(limelightName, index);
    }

    int getTagID() {
        return (int)LimelightHelpers::getFiducialID();
    }

    double getTX() {
        return LimelightHelpers::getTX(limelightName);
    }

    double getTY() {
        return LimelightHelpers::getTY(limelightName);
    }

    TagType getTagType() {
        int tagID = getTagID();
        if (tagID >= 10 && tagID <= 22) {
            return REEF;
        } else if (tagID == 1 || tagID == 2 || tagID == 12 || tagID == 13) {
            return CORALSTATION;
        } else if (tagID == 3 || tagID == 16) {
            return PROCESSOR;
        } else if (tagID == 4 || tagID == 5 || tagID == 14 || tagID == 15) {
            return BARGE;
        } else {
            return REEF;
        }
    }

    double getTagHeight() {
        int tagID = getTagID();
        if (tagData.count(tagID)) {
            return tagData[tagID].height;
        } else {
            return 0;
        }
    }

    double getAngleSetpoint() {
        int tagID = getTagID();
        if (tagData.count(tagID)) {
            return tagData[tagID].angleSetpoint;
        } else {
            return 0;
        }
    }

    double getDistanceToWall() {
        return getTargetPoseRobotSpace().y;
    }

    double getAngleLimelightToTag() {
        if (isTargetDetected()) {
            return limelightMountAngle + getTY();
        } else {
            return 0;
        }
    }

    std::vector<double> getPolarCoords() {
        return {getTX(), getDistanceToWall()};
    }

    std::vector<double> getXYCoords() {
        double angle = getTX() * (3.14153 / 180);
        double dist = getDistanceToWall();
        double x = dist * sin(angle);
        double y = dist * cos(angle);
        return {x, y};
    }

    Pose3d getTargetPoseRobotSpace() {
        std::vector<double> x = LimelightHelpers::getTargetPose_RobotSpace();
        Pose3d output = Pose3d(x);
        double tempY = output.y;
        output.y = output.z;
        output.z = -tempY;
        return output;
    }

    Pose3d getRobotPoseFieldSpace() {
        std::vector<double> x = LimelightHelpers::getBotpose();
        Pose3d output = Pose3d(x);
        double tempY = output.y;
        output.y = output.z;
        output.z = -tempY;
        return output;
    }
};