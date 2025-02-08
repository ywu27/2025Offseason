#pragma once

#include <vector>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include "LimelightHelpers.h"
#include "geometry/Pose3d.h"
#include "Constants.h"

class Limelight {

private:
    std::string limelightName;
    double limelightMountAngle = 30; // Measured in degrees
    double limelightHeight = 5; // Measured in inches
    enum TagType {REEF, CORALSTATION, PROCESSOR, BARGE};
    enum Alliance{RED, BLUE};
    
    double reefTagHeight = 6.875; // Measured in inches
    double coralStationTagHeight = 53.25; // Measured in inches
    double processorTagHeight = 45.875; // Measured in inches
    double bargeTagHeight = 69.0; // Measured in inches
    
    std::vector<int> redReefTargetIDs = {10, 11, 6, 7, 8, 9};
    std::vector<int> blueReefTargetIDs = {21, 20, 19, 18, 17, 22};
    std::vector<int> reefTargetAngles = {180, 240, 300, 0, 60, 120};

    std::vector<int> redCoralStationTargetIDs = {1, 2};
    std::vector<int> blueCoralStationTargetIDs = {12, 13};
    int coralStationTargetAngle1 = 126;
    int coralStationTargetAngle2 = 234;

    int redProcessorTargetIDs = 3;
    int blueProcessorTargetIDs = 16;
    int processorAngle = 270;

    std::vector<int> redBargeTargetIDs = {4, 14};
    std::vector<int> blueBargeTargetIDs = {5, 15};
    int bargeAngle = 0;
    
    Alliance alliance;
    TagType tagType;

public:
    enum Alliance{RED, BLUE};
    enum TagType {REEF, CORALSTATION, PROCESSOR, BARGE};
    //Alliance alliance;
    //TagType tagType;

    Limelight(std::string name, double mountAngle, double heightOffFloor, int allianceOption){
        limelightName = name;
        tagType = REEF;
        
        if (allianceOption == 0){
            alliance = RED;
        }

        else{
            alliance = BLUE;
        }
    }

    bool isTargetDetected() {
        if ((nt::NetworkTableInstance::GetDefault().GetTable("")->GetNumber("tv", 0.0)) == 1)
        {
            frc::SmartDashboard::PutNumber("insidetest", (nt::NetworkTableInstance::GetDefault().GetTable("")->GetNumber("tv", 0.0)));
            return true;
        }
        frc::SmartDashboard::PutNumber("insidetest", (nt::NetworkTableInstance::GetDefault().GetTable("")->GetNumber("tv", 0.0)));
        return false;
    }

    void setPipelineIndex(int index) {
        LimelightHelpers::setPipelineIndex("", index);
    }

    int getTagID() { // returns the ID of the AprilTag
        return (int)LimelightHelpers::getFiducialID();
    }

    double getTX() { // TX
        double tx = LimelightHelpers::getTX("");
        frc::SmartDashboard::PutNumber("Tx", tx);
        return tx;
    }

    double getTY() { // TY
        if (isTargetDetected() == true)
        {
            double ty = LimelightHelpers::getTY("");
            frc::SmartDashboard::PutNumber("Ty", ty);
            return ty;
        }
        return 0.0;
    }

    double isTargetDetected2() {
        return !(getTX()==0);
    }

    // sets the tag height based on the target ID by comparing it to the vectors of target IDs in different positions of the field
    // returns the tag height in inches

    TagType getTagType(){
        //int tagID = 3;
        int tagID = getTagID();
        if(tagID<12){
            alliance=RED;
        }
        else{
            alliance=BLUE;
        }
        switch (alliance) {
            case RED:
                if(tagID<12 && tagID>5){
                    tagType = REEF;
                }
                else if(tagID == 1 || tagID == 2){
                    tagType = CORALSTATION;
                }
                else if(tagID == 3){
                    tagType = PROCESSOR;
                }
                else if(tagID == 4 || tagID == 14){
                    tagType = BARGE;
                }
                break;
            case BLUE:
                if(tagID<23 && tagID>16){
                    tagType = REEF;
                }
                else if(tagID == 12 || tagID == 13){
                    tagType = CORALSTATION;
                }
                else if(tagID == 16){
                    tagType = PROCESSOR;
                }
                else if(tagID == 5 || tagID == 15){
                    tagType = BARGE;
                }
                break;
        }
        return tagType;
    }
    
    double getTagHeight(){

        double tagHeight; //Measured in inches
        tagType = getTagType();
        switch(tagType){
            case REEF:
                tagHeight = reefTagHeight;
                break;
            case CORALSTATION:
                tagHeight = coralStationTagHeight;
                break;
            case PROCESSOR:
                tagHeight = processorTagHeight;
                break;
            case BARGE:
                tagHeight = bargeTagHeight;
                break;
        }
        return tagHeight;

    }
    
    // sets the angle setpoint based on the angle of the field element's tag ID
    double setAngleSetpoint(){
        
        double angleSetpoint = 0;
        tagType = getTagType();
        int tagID = getTagID();
        
        switch(tagType){
            case REEF:
                if (alliance == RED){
                    int redIndexAngle = std::find(redReefTargetIDs.begin(), redReefTargetIDs.end(), tagID) - redReefTargetIDs.begin();
                    angleSetpoint = reefTargetAngles[redIndexAngle];
                }

                else{
                    int blueIndexAngle = std::find(blueReefTargetIDs.begin(), blueReefTargetIDs.end(), tagID) - blueReefTargetIDs.begin();
                    angleSetpoint = reefTargetAngles[blueIndexAngle];
                }
                break;
            
            case CORALSTATION:
                if (tagID == 13 or tagID == 1){
                    angleSetpoint = coralStationTargetAngle1;
                }

                else if (tagID == 12 or tagID == 2){
                    angleSetpoint = coralStationTargetAngle2;
                }
                break;
            
            case PROCESSOR:
                angleSetpoint = processorAngle;
                break;
            
            case BARGE:
                angleSetpoint = bargeAngle;
                break;
        }
        
        return angleSetpoint;
    }
    
    double getDistanceToWall() { // perpendicular distance to wall in meters
        double tagHeight = getTagHeight();
        double ty = LimelightHelpers::getTY("");
        double angleToTagDegrees = limelightMountAngle + ty;
        double angleToTagRadians = angleToTagDegrees * (PI / 180.0);
        double distanceToWall = (tagHeight - limelightHeight) / tan(angleToTagRadians);
        distanceToWall *= 0.0254; //converted to meters
        frc::SmartDashboard::PutNumber("distanceToWall", distanceToWall);
        return distanceToWall;
    }

    double getCurrentAngle() {
        
        //Uses the getTX() function to get the horizontal angle offset from the field element
        //This angle offset is added to the angle of the field element to get the current angle of the robot
        if (isTargetDetected2() == true) {
            //double horizontalDistance = LimelightHelpers::getTX();
            //double distanceFromWall = getDistanceToWall();
            //double angleOffset = atan(horizontalDistance / distanceFromWall);
            double angleOffset = LimelightHelpers::getTX();
            double angleFieldElement = setAngleSetpoint();
            double currentAngle = angleFieldElement + angleOffset;
            return currentAngle;
        }

        else {
            return 0;
        }
    }
    
    double getAngleLimelightToTag() { // TY + limelight mount angle
        if (isTargetDetected() == true)
        {
            double ty = getTY();
            double angleToTagDegrees = limelightMountAngle + ty;
            frc::SmartDashboard::PutNumber("vertical angle", angleToTagDegrees);
            return angleToTagDegrees;
        }
        else
        {
            return 0;
        }
    }

    std::vector<double> getPolarCoords() {
        std::vector<double> polarCoords = {getTX(), getDistanceToWall()};
        return polarCoords;
    }

    std::vector<double> getXYCoords() {
        double angle = getTX() * (PI / 180);
        double x = (getDistanceToWall()) * sin(angle);
        double y = (getDistanceToWall()) * cos(angle);
        std::vector<double> xyCoords = {x, y};
        return xyCoords;
    }

    bool isIn(int object, std::vector<int> inp) { // use to check if current ID is part of target ID list
        for (unsigned int i = 0; i < inp.size(); i++)
        {
            if (inp[i] == object)
            {
                return true;
            }
        }
        return false;
    }

    void setLEDMode(int mode) { // 0 for off / 1 for blink / 2 for on
        nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->PutNumber("ledMode", mode);
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