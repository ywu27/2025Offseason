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
  pigeon.init();
  mSuperstructure.init();

  frc::CameraServer::StartAutomaticCapture();
  // Choosers
  allianceChooser.SetDefaultOption("Red Alliance", redAlliance);
  allianceChooser.AddOption("Blue Alliance", blueAlliance);
  frc::SmartDashboard::PutData("Alliance Color", &allianceChooser);

  // Determines alliance color
  std::string allianceColor = allianceChooser.GetSelected();
  if (allianceColor == "RED") {
    allianceIsRed = true;
  }
  else {
    allianceIsRed = false;
  }

  positionChooser.SetDefaultOption(kAutoStartDefault, kAutoStartDefault);
  positionChooser.AddOption(kAutoStartB, kAutoStartB);
  positionChooser.AddOption(kAutoStartC, kAutoStartC);
  positionChooser.AddOption(kSimpleAuto, kSimpleAuto);
  frc::SmartDashboard::PutData("Auto Start Position", &positionChooser);

  reefChooser.SetDefaultOption(kAutoReefDefault, kAutoReefDefault);
  reefChooser.AddOption(kAutoReefB, kAutoReefB);
  reefChooser.AddOption(kAutoReefC, kAutoReefC);
  reefChooser.AddOption(kAutoReefD, kAutoReefD);
  reefChooser.AddOption(kAutoReefE, kAutoReefE);
  reefChooser.AddOption(kAutoReefF, kAutoReefF);
  reefChooser.AddOption(kOneCoral, kOneCoral);
  frc::SmartDashboard::PutData("Auto Reef Position", &reefChooser);

}

void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutNumber("enc1", mSuperstructure.mElevator.enc.GetPosition());
  frc::SmartDashboard::PutNumber("enc2", mSuperstructure.mElevator.enc2.GetPosition());
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mDrive.enableModules();
  pigeon.pigeon.Reset();

  align.forwardPID.Reset();
  align.strafePID.Reset();
  mDrive.resetPoseEstimator(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  std::string start_pos = positionChooser.GetSelected();
  std::string reef_pos = reefChooser.GetSelected();
  std::string allianceColor = allianceChooser.GetSelected();

  // Auto path choosing
  if(start_pos=="1" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_1A, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_1B, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_1C, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_1D, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_1E, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_1F, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_2A, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_2B, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_2C, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_2D, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_2E, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_2F, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_3A, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_3B, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_3C, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_3D, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_3E, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_3F, allianceIsRed);
  }
  else if (start_pos == "0" && reef_pos == "0") {
    mTrajectory.followPath(Trajectory::MOVE_STRAIGHT, allianceIsRed);
  }
}

void Robot::AutonomousPeriodic()
{
  mLED.Set_Color(frc::Color::kBrown);
}
void Robot::TeleopInit()
{
  mDrive.state = DriveState::Teleop;

  mDrive.enableModules();
  // pigeon.pigeon.SetYaw(units::degree_t(fmod(pigeon.getBoundedAngleCCW().getDegrees() + 180, 360)));
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  if (coralLevel == 1) {
    speedLimiter = 0.8;
  }
  else if (coralLevel ==2 ) {
    speedLimiter = 0.7;
  }
  else if (coralLevel == 3) {
    speedLimiter = 0.5;
  }
  else if (coralLevel == 4) {
    speedLimiter = 0.3;
  }
  else if (coralLevel==5) {
    speedLimiter = 0.9;
  }

  bool fieldOriented = pigeon.pigeon.IsConnected();

  double vx = 0;
  double vy = 0;

  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);
  leftX = xStickLimiter.calculate(leftX) * speedLimiter;
  leftY = yStickLimiter.calculate(leftY) * speedLimiter;
  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  double rot = 0;

  // Driver
  int dPad = ctr.GetPOV();
  bool alignPV = ctr.GetR2Button();
  bool intakeCoral = ctr.GetTriangleButton();
  bool scoreCoral = ctr.GetCrossButtonPressed();
  
  // Co-driver
  bool coralIntake = ctrOperator.GetR1Button();
  bool level1 = ctrOperator.GetCrossButton();
  bool level2 = ctrOperator.GetSquareButton();
  bool level3 = ctrOperator.GetCircleButton();
  bool level4 = ctrOperator.GetTriangleButton();

  bool resetGyro = (ctrOperator.GetPOV() == 0) || (ctr.GetPOV() == 0);

  bool zeroElevator = ctrOperator.GetL1ButtonPressed();
  mSuperstructure.mElevator.motor.Set(-ctrOperator.GetLeftY() * 1);
  mSuperstructure.mElevator.motor2.Set(-ctrOperator.GetLeftY() * 1);

  int dPadOperator = ctrOperator.GetPOV();
  
  // Driving Modes
  double offSet = 0;
  double targetDistance = 0; // CHECK THIS
  double zeroSetpoint = 0;

  // frc::Color detectedColor = color.GetColor();
  // bool red = (detectedColor.red < 0.29) && (detectedColor.red > 0.255);
  // bool green = (detectedColor.green < 0.495) && (detectedColor.green > 0.47);
  // bool blue = (detectedColor.blue < 0.27) && (detectedColor.blue > 0.23);
  // bool coralIn = red && green && blue;

  // frc::SmartDashboard::PutBoolean("coralIN", coralIn);

  // if (coralIn) {
  //   mLED.Set_Color(frc::Color::kGreen);
  // }

  if(dPadOperator==90) {
    coralSide = "right";
  }
  else if(dPadOperator==270) {
    coralSide = "left";
  }

  if(level1) {
    coralLevel = 1;
    mSuperstructure.mElevator.setState(1);
    elevatorLevel = "Level 1";
  }
  else if(level2) {
    coralLevel = 2;
    mSuperstructure.mElevator.setState(2);
    elevatorLevel = "Level 2";
  }
  else if(level3) {
    coralLevel = 3;
    mSuperstructure.mElevator.setState(3);
    elevatorLevel = "Level 3";
  }
  else if(level4) {
    coralLevel = 4;
    mSuperstructure.mElevator.setState(4);
    elevatorLevel = "Level 4";
  }
  else if (coralIntake) {
    coralLevel = 5;
    elevatorLevel = "Coral Station";
    mSuperstructure.mElevator.setState(5);
  }
  
  if (ctr.GetR2ButtonPressed()) {
    align.forwardPID.Reset();
    align.strafePID.Reset();
  }
  else if (alignPV && cameraFront.isTargetDetected()) { // Alignment Mode
    targetDistance = 0.275;
    offSet = -0.07;
    ChassisSpeeds speeds = align.autoAlignPV(cameraFront, targetDistance, offSet);
    vx = speeds.vxMetersPerSecond;
    vy = speeds.vyMetersPerSecond;
    fieldOriented = false;

    mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
    mHeadingController.setSetpoint(cameraFront.getAngleSetpoint());
    rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
  }
  else if (alignPV && cameraBack.isTargetDetected() && cameraBack.isCoralStation()) {
      targetDistance = 0.2;

      ChassisSpeeds speeds = align.autoAlignPV(cameraBack, targetDistance, 0.07);
      vx = speeds.vxMetersPerSecond;
      vy = speeds.vyMetersPerSecond;
      fieldOriented = false;
      
      mHeadingController.setSetpoint(cameraBack.getAngleSetpoint());
      mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
      rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
  }
  // else if (alignPV && limelight2.isTargetDetected()) {
  //   offSet = 0;
  //   targetDistance = 0.4; //set this
  //     ChassisSpeeds speeds = align.autoAlign(limelight2, targetDistance, 0);
  //     vx = speeds.vxMetersPerSecond;
  //     vy = speeds.vyMetersPerSecond;
  //     fieldOriented = false;
      
  //     mHeadingController.setSetpoint(limelight2.getAngleSetpoint());
  //     mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
  //     rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
  // }
  else // Normal driving mode
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
    vx = leftX * moduleMaxFPS;
    vy = leftY * moduleMaxFPS;
    rot = rightX * moduleMaxRot * 2;
  }

  // Gyro Resets
  if (resetGyro) {
    pigeon.pigeon.Reset();
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(vx, vy, rot),
      pigeon.getBoundedAngleCCW(),
      fieldOriented,
      cleanDriveAccum);
  mDrive.updateOdometry();

  // Superstructure
  if (intakeCoral) {
    mLED.Set_Color(frc::Color::kRed);

    if (coralLevel != 4 && coralLevel != 1) {
      mSuperstructure.intakeCoral();
    }
    else if (coralLevel == 1) {
      mSuperstructure.mEndEffector.scoreL1();
    }
    else if (coralLevel == 4) {
      mSuperstructure.mEndEffector.scoreL4();
    }
  }
  else if (zeroElevator) {
    mSuperstructure.mElevator.zero();
  }
  else {
    mSuperstructure.mEndEffector.setState(EndEffector::STOP);
    // mSuperstructure.mElevator.setState(coralLevel);
    mLED.Set_Color(frc::Color::kGreen);
  }

  if (cameraFront.isTargetDetected()) {
    mLED.Set_Color(frc::Color::kBlue);
  }
}

void Robot::DisabledInit()
{
  mDrive.state = DriveState::Disabled;
  mDrive.disableModules();
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