/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/estimation/VisionEstimation.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/simulation/VisionTargetSim.h>
#include <photon/targeting/PhotonPipelineResult.h>

#include <functional>
#include <limits>
#include <memory>
#include <algorithm>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include "PhotonVision.h"
#include "Constants.h"

class Vision {
 public:
  /**
   * @param estConsumer Lamba that will accept a pose estimate and pass it to
   * your desired SwerveDrivePoseEstimator.
   */
  Vision(std::function<void(frc::Pose2d, units::second_t,
                            Eigen::Matrix<double, 3, 1>)>
             estConsumer, PhotonVision &camera, PhotonVision &camera1, PhotonVision &camera2, PhotonVision &camera3)
      : estConsumer{estConsumer}, camera(camera), camera1(camera1), camera2(camera2), camera3(camera3) {
    photonEstimator.SetMultiTagFallbackStrategy(
        photon::PoseStrategy::LOWEST_AMBIGUITY);
  }

  photon::PhotonPipelineResult GetLatestResult() { return m_latestResult; }

  void Periodic() {
    std::array<double, 4> cameraAmbiguity = {camera.getAmbiguity(), camera1.getAmbiguity(), camera2.getAmbiguity(), camera3.getAmbiguity()};
    auto min = std::min_element(cameraAmbiguity.begin(), cameraAmbiguity.end());
    double minAmb = *min;
    int index = 0;
    std::vector<photon::PhotonPipelineResult> camResults;
    // i = num of cameras dedicated to pose estimation
    // Find lowest ambiguity camera
    for (int i = 0; i < 4; i++) {
      if ((cameraAmbiguity[i] + 0.01) == minAmb || ((cameraAmbiguity[i] - 0.01) == minAmb)) {
        index = i;
      }
    }
    
    // Get cam results for lowest ambiguity camera (looping will take up a lot of memory usage)
    if (index == 0) {
      camResults = camera.camera.GetAllUnreadResults();
    }
    if (index == 1) {
      camResults = camera1.camera.GetAllUnreadResults();
    }
    if (index == 2) {
      camResults = camera2.camera.GetAllUnreadResults();
    }
    if (index == 3) {
      camResults = camera3.camera.GetAllUnreadResults();
    }
    
    // Run each new pipeline result through our pose estimator
    for (const auto& result : camResults) {
      // cache result and update pose estimator
      auto visionEst = photonEstimator.Update(result);
      m_latestResult = result;

      if (visionEst) {
        estConsumer(visionEst->estimatedPose.ToPose2d(), visionEst->timestamp,
                    GetEstimationStdDevs(visionEst->estimatedPose.ToPose2d()));
      }
    }
  }

  Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose) {
    Eigen::Matrix<double, 3, 1> estStdDevs = {4, 4, 8};
    auto targets = GetLatestResult().GetTargets();
    int numTags = 0;
    units::meter_t avgDist = 0_m;
    for (const auto& tgt : targets) {
      auto tagPose =
          photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());
      if (tagPose) {
        numTags++;
        avgDist += tagPose->ToPose2d().Translation().Distance(
            estimatedPose.Translation());
      }
    }
    if (numTags == 0) {
      return estStdDevs;
    }
    avgDist /= numTags;
    if (numTags > 1) {
      estStdDevs = {0.5, 0.5, 1};
    }
    if (numTags == 1 && avgDist > 4_m) {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max())
                       .finished();
    } else {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }
    return estStdDevs;
  }

 private:
  frc::AprilTagFieldLayout apriltagField = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
  photon::PhotonPoseEstimator photonEstimator{
      apriltagField, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      frc::Transform3d(0.0_m, 0.0_m, 0.0_m, frc::Rotation3d(0.0_deg, 0.0_deg, 0.0_deg))};
  PhotonVision &camera;
  PhotonVision &camera1;
  PhotonVision &camera2;
  PhotonVision &camera3;
  std::unique_ptr<photon::VisionSystemSim> visionSim;
  std::unique_ptr<photon::SimCameraProperties> cameraProp;
  std::shared_ptr<photon::PhotonCameraSim> cameraSim;

  // The most recent result, cached for calculating std devs
  photon::PhotonPipelineResult m_latestResult;
  std::function<void(frc::Pose2d, units::second_t, Eigen::Matrix<double, 3, 1>)>
      estConsumer;
};