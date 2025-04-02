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
    
    struct TagInfo {
        double height;
        int angleSetpoint;
    };
    
    std::map<int, TagInfo> tagData = {
        {1, {53.25, 234}}, {2, {53.25, 126}}, {3, {45.875, 90}}, {4, {69.0, 0}}, {5, {69.0, 0}},
        {10, {6.875, 180}}, {11, {6.875, 120}}, {6, {6.875, 60}}, {7, {6.875, 0}}, {8, {6.875, 300}}, {9, {6.875, 240}},
        {12, {53.25, 126}}, {13, {53.25, 234}}, {14, {69.0, 0}}, {15, {69.0, 0}}, {16, {45.875, 90}},
        {17, {6.875, 300}}, {18, {6.875, 0}}, {19, {6.875, 60}}, {20, {6.875, 120}}, {21, {6.875, 180}}, {22, {6.875, 240}}
    };

public:
    enum Alliance{RED, BLUE};
    enum TagType {REEF, CORALSTATION, PROCESSOR, BARGE};
    
    Alliance alliance;
    TagType tagType;

    Limelight(std::string name){
        limelightName = name;
    }

    bool isTargetDetected() {
        if (getTX()==0 && getTY()==0) {
            return false;
        }
        return true;
    }

    void setPipelineIndex(int index) {
        LimelightHelpers::setPipelineIndex(limelightName, index);
    }

    int getTagID() {
        if (isTargetDetected()) {
            return (int)LimelightHelpers::getFiducialID();
        }
    }

    double getTX() {
        return LimelightHelpers::getTX(limelightName);
    }

    double getTY() {
        return LimelightHelpers::getTY(limelightName);
    }

    std::string getName() {
        return limelightName;
    }

    TagType getTagType() {
        if(isTargetDetected()) {
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
    }

    double getTagHeight() {
        if (isTargetDetected()) {
            int tagID = getTagID();
            if (tagData.count(tagID)) {
                return tagData[tagID].height;
            } else {
                return 0;
            }
        }
    }

    double getAngleSetpoint() {
        if (isTargetDetected()) {
            int tagID = getTagID();
            if (tagData.count(tagID)) {
                return tagData[tagID].angleSetpoint;
            } else {
                return 0;
            }
        }   
    }

    double getDistanceToWall() {
        if (isTargetDetected()) {
            return getTargetPoseRobotSpace().y;
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
        std::vector<double> x = LimelightHelpers::getTargetPose_RobotSpace(limelightName);
        Pose3d output = Pose3d(x);
        double tempY = output.y;
        output.y = output.z;
        output.z = -tempY;
        return output;
    }

    Pose3d getRobotPoseFieldSpace() {
        std::vector<double> x = LimelightHelpers::getBotpose(limelightName);
        Pose3d output = Pose3d(x);
        double tempY = output.y;
        output.y = output.z;
        output.z = -tempY;
        return output;
    }
};