// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <string>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit()
{
  mDrive.initModules();
  limelight.setPipelineIndex(0);
  //frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic()
{
  
  limelight.isTargetDetected();
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mGyro.init();
  mDrive.enableModules();
 
  // if (frc::DriverStation::IsDSAttached()) {
  //   mTraj.isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed; // checks for alliance color
  // }
  // if (limelight.targetDetected()) {
  //   mTraj.startPose = mLimelight.getRobotPoseFieldSpace();
  //   mTraj.receivedPose = true;
  // } else {
  //   mTraj.receivedPose = false;
  // }
}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{
  mDrive.state = DriveState::Teleop;
  frc::SmartDashboard::PutNumber("tx", limelight.getTX());
  limelight.setPipelineIndex(0);
  limelight.isTargetDetected();
  limelight.setLEDMode(0);

  mDrive.enableModules();
  mGyro.init();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  bool fieldOriented = false;
  fieldOriented = mGyro.gyro.IsConnected();

  frc::SmartDashboard::PutBoolean("aligned?", align.isAligned(limelight));

  auto startTime = frc::Timer::GetFPGATimestamp();
  double vx = 0;
  double vy = 0;

  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);

  leftX = xStickLimiter.calculate(leftX); 
  leftY = yStickLimiter.calculate(leftY);

  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);

  //int dPad = ctrOperator.GetPOV();
  bool rumbleController = false;

  // Driver Information

  // Teleop States
  double rot = rightX * moduleMaxRot * 2;

  //Decide drive modes
  double zeroSetpoint = 0;

  if (ctr.GetR2Button()&&limelight.isTargetDetected2()) {
    ChassisSpeeds speeds = align.autoAlign(limelight, mHeadingController, 0.75);
    frc::SmartDashboard::PutNumber("strafe", speeds.vyMetersPerSecond);
    vx = speeds.vyMetersPerSecond;
    vy = speeds.vxMetersPerSecond;
    frc::SmartDashboard::PutNumber("vx", vx);
    frc::SmartDashboard::PutNumber("vy", vy);
    fieldOriented = false;
    zeroSetpoint = 0;
    frc::SmartDashboard::PutNumber("Gyro position", mGyro.getBoundedAngleCCW().getDegrees());
    mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
    mHeadingController.setSetpoint(zeroSetpoint);
    rot = mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
  }
  else // Normal driving mode
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
    vx = leftX * moduleMaxFPS;
    vy = leftY * moduleMaxFPS;
  }
  // Gyro Resets
  if (ctr.GetCrossButtonReleased()) {
    //mGyro.init();

    //Setting gyro to the current angle of the robot
    mGyro.setYaw(limelight.getCurrentAngle());
  }

  if (align.isAligned(limelight)) {
    mGyro.setYaw(0);
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(vx, vy, -rot),
      mGyro.getBoundedAngleCCW(),
      fieldOriented,
      cleanDriveAccum);
  mDrive.updateOdometry();
  frc::SmartDashboard::PutNumber("Gyro position", mGyro.getBoundedAngleCCW().getDegrees());
  frc::SmartDashboard::PutNumber("vx", vx);
  frc::SmartDashboard::PutNumber("vy", vy);
  frc::SmartDashboard::PutNumber("driveX", mDrive.getOdometryPose().X().value());
  frc::SmartDashboard::PutNumber("driveY", mDrive.getOdometryPose().Y().value());
}

void Robot::DisabledInit()
{
  mDrive.state = DriveState::Disabled;
  mDrive.stopModules();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif