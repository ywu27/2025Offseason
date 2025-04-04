#pragma once

#include <frc/controller/PIDController.h>
#include "Constants.h"

#include "photon/PhotonCamera.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "photon/estimation/VisionEstimation.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/targeting/PhotonPipelineResult.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

class PhotonVision {
private:
    double targetDistanceMeters;
    double targetYaw;
    double targetFiducial;
    double targetxMeters;
    
    double cameraPitchRadians;
    double cameraHeightMeters;
    double cameraPositionOffsetxMeters;
    double cameraPositionOffsetyMeters;

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
    frc::AprilTagFieldLayout apriltagField = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
    photon::PhotonPoseEstimator poseEstimator{
      apriltagField, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      frc::Transform3d(0.0_m, 0.0_m, 0.0_m, frc::Rotation3d(0.0_deg, 0.0_deg, 0.0_deg))};

    enum TagType {REEF, CORALSTATION, PROCESSOR, BARGE};

    photon::PhotonCamera camera;
    
    PhotonVision(std::string name)
        : camera(name),
          cameraPitchRadians(0.0),
          cameraHeightMeters(1.0), // FIX THIS
          cameraPositionOffsetxMeters(0.0),
          cameraPositionOffsetyMeters(0.0) {}

    void init() {
        camera.SetPipelineIndex(0);
    }

    bool isTargetDetected() {
        return camera.GetLatestResult().HasTargets();
    }

    int getTagID() {
        return camera.GetLatestResult().GetBestTarget().GetFiducialId();
    }

    void getInformationOfSpecificTargetFiducial(auto targetsSpan, int fiducial) {
        for (auto target : targetsSpan) {
            if (target.GetFiducialId() == fiducial) {
                targetDistanceMeters = target.GetBestCameraToTarget().Translation().Norm().value();
                targetYaw = target.GetBestCameraToTarget().Rotation().Z().value();
                targetFiducial = target.GetFiducialId();
                targetxMeters = target.GetBestCameraToTarget().Translation().X().value();
            }
        }
    }

    TagType getTagType() {
        if(camera.GetLatestResult().HasTargets()) {
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

    bool isCoralStation() {
        return (getTagType() == TagType::CORALSTATION);
    }

    bool isReef() {
        return (getTagType() == TagType::REEF);
    }

    double getDistanceToTarget() {
        return camera.GetLatestResult().GetBestTarget().GetBestCameraToTarget().X().value();
    }

    double getStrafeDistancetoTarget() {
        return camera.GetLatestResult().GetBestTarget().GetBestCameraToTarget().Y().value();
    }

    double getYaw() {
        return camera.GetLatestResult().GetBestTarget().GetYaw();
    }

    frc::Pose2d returnPoseEstimate() {
        std::optional<photon::EstimatedRobotPose> visionCache;

        for (const auto& result : camera.GetAllUnreadResults()) {
            if (camera.GetLatestResult().HasTargets()) {
                visionCache = poseEstimator.Update(result);
                photon::PhotonPipelineResult latestResult = result;
            }
        }
        return visionCache->estimatedPose.ToPose2d();
    }
};