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
  frc::SmartDashboard::PutBoolean("isCoralDetected", mSuperstructure.mEndEffector.cSensor.isTarget());
  frc::SmartDashboard::PutNumber("proximmitry", mSuperstructure.mEndEffector.cSensor.GetProximity());
  frc::SmartDashboard::PutNumber("red", mSuperstructure.mEndEffector.cSensor.getcolor().red);
  frc::SmartDashboard::PutNumber("green", mSuperstructure.mEndEffector.cSensor.getcolor().green);
  frc::SmartDashboard::PutNumber("blue", mSuperstructure.mEndEffector.cSensor.getcolor().blue);
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
  if (allianceColor == "RED") {
    allianceIsRed = true;
  }
  else {
    allianceIsRed = false;
  }
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
  pigeon.pigeon.Reset();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}
void Robot::TeleopPeriodic()
{
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
  bool scoreCoral = ctr.GetCrossButtonPressed();
  
  // Co-driver
  bool coralIntake = ctrOperator.GetR1Button();
  bool level1 = ctrOperator.GetCrossButton();
  bool level2 = ctrOperator.GetSquareButton();
  bool level3 = ctrOperator.GetCircleButton();
  bool level4 = ctrOperator.GetTriangleButton();

  bool resetGyro = (ctrOperator.GetPOV() == 0);

  bool zeroElevator = ctrOperator.GetL1ButtonPressed();
  mSuperstructure.mElevator.motor.Set(-ctrOperator.GetLeftY() * 1);
  mSuperstructure.mElevator.motor2.Set(-ctrOperator.GetLeftY() * 1);

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
    // mSuperstructure.mElevator.setState(1);
    elevatorLevel = "Level 1";
  }
  else if(level2) {
    coralLevel = 2;
    // mSuperstructure.mElevator.setState(2);
    elevatorLevel = "Level 2";
  }
  else if(level3) {
    coralLevel = 3;
    // mSuperstructure.mElevator.setState(3);
    elevatorLevel = "Level 3";
  }
  else if(level4) {
    coralLevel = 4;
    // mSuperstructure.mElevator.setState(4);
    elevatorLevel = "Level 4";
  }
  else if (coralIntake) {
    coralLevel = 5;
    elevatorLevel = "Coral Station";
    // mSuperstructure.mElevator.setState(5);
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
  // else if (mSuperstructure.mElevator.limitSwitch.Get()) {
  //   mSuperstructure.mEndEffector.setState(EndEffector::STOP);
  //   mSuperstructure.mElevator.zero();
  // }
  else {
    mSuperstructure.mEndEffector.setState(EndEffector::STOP);
    
  mSuperstructure.mElevator.setState(coralLevel);
    mLED.Set_Color(frc::Color::kDarkOrange);
  }
  
  // Smart Dashboard
  // frc::SmartDashboard::PutString("Elevator Stage", elevatorLevel);
  frc::SmartDashboard::PutNumber("Gyro CW", pigeon.getBoundedAngleCW().getDegrees());
  // frc::SmartDashboard::PutBoolean("Aligned?", align.isAligned(limelight1));
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