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
  mGyro.init();
  climber.init();
  elevator.init();
  // limelight.setPipelineIndex(0);
  // frc::CameraServer::StartAutomaticCapture();
  start_pos_chooser.SetDefaultOption(kAutoStartDefault, kAutoStartDefault);
  start_pos_chooser.AddOption(kAutoStartB, kAutoStartB);
  start_pos_chooser.AddOption(kAutoStartC, kAutoStartC);
  frc::SmartDashboard::PutData("Auto Start Position", &start_pos_chooser);

  reef_pos_chooser.SetDefaultOption(kAutoReefDefault, kAutoReefDefault);
  reef_pos_chooser.AddOption(kAutoReefB, kAutoReefB);
  reef_pos_chooser.AddOption(kAutoReefC, kAutoReefC);
  reef_pos_chooser.AddOption(kAutoReefD, kAutoReefD);
  reef_pos_chooser.AddOption(kAutoReefE, kAutoReefE);
  reef_pos_chooser.AddOption(kAutoReefF, kAutoReefF);
  frc::SmartDashboard::PutData("Auto Reef Position", &reef_pos_chooser);

}

void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mGyro.init();
  mDrive.enableModules();
  
  std::string start_pos = start_pos_chooser.GetSelected();
  std::string reef_pos = reef_pos_chooser.GetSelected();

  // if(start_pos=="1" && reef_pos=="A") {
  //   mTrajectory.followPath(Trajectory::auto_1A, false);
  // }
  // else if(start_pos=="1" && reef_pos=="B") {
  //   mTrajectory.followPath(Trajectory::auto_1B, false);
  // }
  // else if(start_pos=="1" && reef_pos=="C") {
  //   mTrajectory.followPath(Trajectory::auto_1C, false);
  // }
  // else if(start_pos=="1" && reef_pos=="D") {
  //   mTrajectory.followPath(Trajectory::auto_1D, false);
  // }
  // else if(start_pos=="1" && reef_pos=="E") {
  //   mTrajectory.followPath(Trajectory::auto_1E, false);
  // }
  // else if(start_pos=="1" && reef_pos=="F") {
  //   mTrajectory.followPath(Trajectory::auto_1F, false);
  // }
  // else if(start_pos=="2" && reef_pos=="A") {
  //   mTrajectory.followPath(Trajectory::auto_2A, false);
  // }
  // else if(start_pos=="2" && reef_pos=="B") {
  //   mTrajectory.followPath(Trajectory::auto_2B, false);
  // }
  // else if(start_pos=="2" && reef_pos=="C") {
  //   mTrajectory.followPath(Trajectory::auto_2C, false);
  // }
  // else if(start_pos=="2" && reef_pos=="D") {
  //   mTrajectory.followPath(Trajectory::auto_2D, false);
  // }
  // else if(start_pos=="2" && reef_pos=="E") {
  //   mTrajectory.followPath(Trajectory::auto_2E, false);
  // }
  // else if(start_pos=="2" && reef_pos=="F") {
  //   mTrajectory.followPath(Trajectory::auto_2F, false);
  // }
  // else if(start_pos=="3" && reef_pos=="A") {
  //   mTrajectory.followPath(Trajectory::auto_3A, false);
  // }
  // else if(start_pos=="3" && reef_pos=="B") {
  //   mTrajectory.followPath(Trajectory::auto_3B, false);
  // }
  // else if(start_pos=="3" && reef_pos=="C") {
  //   mTrajectory.followPath(Trajectory::auto_3C, false);
  // }
  // else if(start_pos=="3" && reef_pos=="D") {
  //   mTrajectory.followPath(Trajectory::auto_3D, false);
  // }
  // else if(start_pos=="3" && reef_pos=="E") {
  //   mTrajectory.followPath(Trajectory::auto_3E, false);
  // }
  // else if(start_pos=="3" && reef_pos=="F") {
  //   mTrajectory.followPath(Trajectory::auto_3F, false);
  // }
  // else {
  //   mTrajectory.followPath(Trajectory::auto_1A, false);
  // }
}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{
  mDrive.state = DriveState::Teleop;

  mDrive.enableModules();
  mGyro.init();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  auto startTime = frc::Timer::GetFPGATimestamp();
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
  bool driveTranslating = !(leftX == 0 && leftY == 0);
  bool driveTurning = !(rightX == 0);
  double rot = rightX * moduleMaxRot * 2;

  //Decide drive modes
  // if (ctr.GetTriangleButton()) // ALIGN(scoring) mode
  // {
  //     Pose3d target = limelight.getTargetPoseRobotSpace();
  //     frc::SmartDashboard::PutNumber("target y", target.y);
  //     frc::SmartDashboard::PutNumber("target x", target.x);
  //     double angleOffset = limelight.getTX();
  //     double zeroSetpoint = 0;
  //     if (angleOffset>0) {
  //       zeroSetpoint = mGyro.getBoundedAngleCW().getDegrees() + angleOffset;
  //     }
  //     else {
  //       zeroSetpoint = mGyro.getBoundedAngleCCW().getDegrees() - angleOffset;
  //     }
  //     //frc::SmartDashboard::PutNumber("steer encoder position", mDrive.mFrontLeft.steerEnc.getAbsolutePosition().getDegrees());
  //     frc::SmartDashboard::PutNumber("Gyro position", mGyro.getBoundedAngleCCW().getDegrees());
  //     mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
  //     mHeadingController.setSetpoint(zeroSetpoint);
  // }
  // else // Normal driving mode
  // {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  // }

  // Output heading controller if used
  if (mHeadingController.getHeadingControllerState() != SwerveHeadingController::OFF) {
    rot = mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
  }

  frc::SmartDashboard::PutNumber("encoder", elevator.motor2.GetEncoder().GetPosition());
  if (ctr.GetR1Button()) {
    elevator.motor2.Set(0.1);
    elevator.motor.Set(0.1);
  }
  else if (ctr.GetR2Button()) {
    elevator.motor2.Set(-0.1);
    elevator.motor.Set(-0.1);
  }
  else {
    elevator.motor2.Set(0);
    elevator.motor.Set(0);
  }

  if (ctr.GetCircleButton()) {
    climber.climb();
  }
  else if (ctr.GetTriangleButton()) {
    climber.reverse();
  }
  else {
    climber.disable();
  }

  // Gyro Resets
  if (ctr.GetCrossButtonReleased())
  {
    mGyro.init();
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(leftX * moduleMaxFPS, leftY * moduleMaxFPS, rot),
      mGyro.getBoundedAngleCCW(),
      mGyro.gyro.IsConnected(),
      cleanDriveAccum);
  mDrive.updateOdometry();
  frc::SmartDashboard::PutNumber("driveX", mDrive.getOdometryPose().X().value());
  frc::SmartDashboard::PutNumber("driveY", mDrive.getOdometryPose().Y().value());
  // frc::SmartDashboard::PutBoolean("testtarget", limelight.isTargetDetected());
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