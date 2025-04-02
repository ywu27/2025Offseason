// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <string>

void Robot::RobotInit()
{
  mDrive.initModules();
  pigeon.init();
  mSuperstructure.init();
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
  pigeon.pigeon.Reset();
  mDrive.enableModules();
  
  std::string start_pos = start_pos_chooser.GetSelected();
  std::string reef_pos = reef_pos_chooser.GetSelected();

  if(start_pos=="1" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_1A, false);
  }
  else if(start_pos=="1" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_1B, false);
  }
  else if(start_pos=="1" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_1C, false);
  }
  else if(start_pos=="1" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_1D, false);
  }
  else if(start_pos=="1" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_1E, false);
  }
  else if(start_pos=="1" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_1F, false);
  }
  else if(start_pos=="2" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_2A, false);
  }
  else if(start_pos=="2" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_2B, false);
  }
  else if(start_pos=="2" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_2C, false);
  }
  else if(start_pos=="2" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_2D, false);
  }
  else if(start_pos=="2" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_2E, false);
  }
  else if(start_pos=="2" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_2F, false);
  }
  else if(start_pos=="3" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_3A, false);
  }
  else if(start_pos=="3" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_3B, false);
  }
  else if(start_pos=="3" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_3C, false);
  }
  else if(start_pos=="3" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_3D, false);
  }
  else if(start_pos=="3" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_3E, false);
  }
  else if(start_pos=="3" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_3F, false);
  }
  else {
    mTrajectory.followPath(Trajectory::auto_1A, false);
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
  pigeon.pigeon.Reset();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
  mLED.Set_Color(frc::Color::kDarkOrange);

  double speedLimiter = mSuperstructure.speedLimiter();
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
  bool alignLimelight = ctr.GetR2Button();
  bool intakeCoral = ctr.GetTriangleButton();
  bool scoreCoral = ctr.GetCrossButton();
  
  // Co-driver
  bool coralIntake = ctrOperator.GetR1Button();
  bool level1 = ctrOperator.GetCrossButtonPressed();
  bool level2 = ctrOperator.GetSquareButtonPressed();
  bool level3 = ctrOperator.GetCircleButtonPressed();
  bool level4 = ctrOperator.GetTriangleButtonPressed();

  bool resetGyro = (ctrOperator.GetPOV() == 0);

  bool zeroElevator = ctrOperator.GetL1ButtonPressed();
  mSuperstructure.mElevator.motor.Set(-ctrOperator.GetLeftY() * 0.5);
  mSuperstructure.mElevator.motor2.Set(-ctrOperator.GetLeftY() * 0.5);

  int dPadOperator = ctrOperator.GetPOV();
  
  // Driving Modes
  double offSet = 0;
  double targetDistance = 0; // CHECK THIS
  double zeroSetpoint = 0;


  if(dPadOperator==90) {
    coralSide = "right";
  }
  else if(dPadOperator==270) {
    coralSide = "left";
  }

  if(level1) {
    coralLevel = 1;
    elevatorLevel = "Level 1";
  }
  else if(level2) {
    coralLevel = 2;
    elevatorLevel = "Level 2";
  }
  else if(level3) {
    coralLevel = 3;
    elevatorLevel = "Level 3";
  }
  else if(level4) {
    coralLevel = 4;
    elevatorLevel = "Level 4";
  }
  else if (coralIntake) {
    coralLevel = 5;
    elevatorLevel = "Coral Station";
  }
  else if ((mSuperstructure.mElevator.enc.GetPosition() < (mSuperstructure.mElevator.levelHeight[coralLevel] + 0.2)) && (mSuperstructure.mElevator.enc.GetPosition() > (mSuperstructure.mElevator.levelHeight[coralLevel] - 0.2))) {
    elevatorLevel = "Transitioning";
  }

  if (alignLimelight && limelight1.isTargetDetected()) { // Alignment Mode
    offSet = 0.0381; // meters
    targetDistance = 1;
    zeroSetpoint = limelight1.getAngleSetpoint();
    ChassisSpeeds speeds = align.autoAlign(limelight1, targetDistance, offSet);
    vx = speeds.vxMetersPerSecond;
    vy = speeds.vyMetersPerSecond;
    fieldOriented = false;
    mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
    // if (limeligh1.getTX() > 0) {
    //   zeroSetpoint = pigeon.getBoundedAngleCW().getDegrees() + angleOffset;
    // }
    // else {
    //   zeroSetpoint = pigeon.getBoundedAngleCCW().getDegrees() - angleOffset;
    // }
    zeroSetpoint = limelight1.getAngleSetpoint();
    mHeadingController.setSetpoint(zeroSetpoint);
    rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
  }

  else if (alignLimelight && limelight2.isTargetDetected()) { // Alignment Mode
    offSet = 0.0381; // meters
    targetDistance = 1;
    zeroSetpoint = limelight2.getAngleSetpoint();
    ChassisSpeeds speeds = align.autoAlign(limelight2, targetDistance, offSet);
    vx = speeds.vxMetersPerSecond;
    vy = speeds.vyMetersPerSecond;
    fieldOriented = false;
    // if (limelight2.getTX() > 0) {
    //   zeroSetpoint = pigeon.getBoundedAngleCW().getDegrees() + angleOffset;
    // }
    // else {
    //   zeroSetpoint = pigeon.getBoundedAngleCCW().getDegrees() - angleOffset;
    // }
    zeroSetpoint = limelight2.getAngleSetpoint();
    mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
    mHeadingController.setSetpoint(zeroSetpoint);
    rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
  }
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
    mSuperstructure.intakeCoral();
  }
  else if (mSuperstructure.mEndEffector.cSensor.isTarget()) {
    mLED.Set_Color(frc::Color::kGreen);
  }
  else if (align.isAligned(limelight1)) {
    mLED.Set_Color(frc::Color::kBlue);
  }
  else if (scoreCoral) {
    mSuperstructure.scoreCoral();
  }
  else if (zeroElevator) {
    mSuperstructure.mElevator.zero();
  }
  else if (mSuperstructure.mElevator.limitSwitch.Get()) {
    mSuperstructure.mEndEffector.setState(EndEffector::STOP);
    mSuperstructure.mElevator.zero();
  }
  else {
    mSuperstructure.mEndEffector.setState(EndEffector::STOP);
    mLED.Set_Color(frc::Color::kWhite);
  }

  mSuperstructure.mElevator.setState(coralLevel);

  // Smart Dashboard
  frc::SmartDashboard::PutString("Elevator Stage", elevatorLevel);
  frc::SmartDashboard::PutNumber("Gyro CW", pigeon.getBoundedAngleCW().getDegrees());
  frc::SmartDashboard::PutBoolean("Aligned?", align.isAligned(limelight1));
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