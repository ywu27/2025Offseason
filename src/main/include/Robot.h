
#pragma once

#include <frc/TimedRobot.h>
#include <frc/PS5Controller.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Joystick.h>

#include "util/ControlUtil.h"
#include "sensors/NavX.h"
#include "swerve/SwerveHeadingController.h"
#include "util/TimeDelayedBool.h"
#include "sensors/Limelight.h"
#include "util/SlewRateLimiter.h"
#include "control/PowerModule.h"
#include "SwerveDrive.h"
#include "util/TimeDelayButton.h"
#include <ctre/phoenix6/CANBus.hpp>
#include "Trajectory.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  // Modules/Devices
  frc::PS5Controller ctr = frc::PS5Controller(0);
  //frc::PS5Controller ctrOperator = frc::PS5Controller(1);
  NavX mGyro = NavX();
  SwerveDrive mDrive = SwerveDrive(mGyro);
  pathplanner::RobotConfig pathConfig = pathplanner::RobotConfig::fromGUISettings();
  frc::SendableChooser<Trajectory::autos> mChooser;
  Trajectory mTrajectory = Trajectory(mDrive, mGyro, limelight, pathConfig);

  //CANivore
  ctre::phoenix6::CANBus canbus{"Drivetrain"};
  ctre::phoenix6::CANBus::CANBusStatus canInfo = canbus.GetStatus();
  float busUtil = canInfo.BusUtilization;
  
  //Limelight
  Limelight limelight = Limelight("one", 30, 5);
  Limelight limelight2 = Limelight("two", 30, 5);

  // Teleop Controls
  float ctrPercent = 1.0;
  float boostPercent = 0.9;
  double ctrPercentAim = 0.3;
  TimeDelayButton snapRobotToGoal;
  bool scoreAmp = false;
  bool liftElev = false;
  bool cleanDriveAccum = true;

  // Controllers
  SwerveHeadingController mHeadingController = SwerveHeadingController(-4.0, 4.0);
  SlewRateLimiter xStickLimiter = SlewRateLimiter(ctrSlewRate);
  SlewRateLimiter yStickLimiter = SlewRateLimiter(ctrSlewRate);

  //autochooser
 frc::SendableChooser<std::string> start_pos_chooser;
 const std::string kAutoStartDefault = "1";
 const std::string kAutoStartB = "2";
 const std::string kAutoStartC = "3";

 frc::SendableChooser<std::string> reef_pos_chooser;
 const::std::string kAutoReefDefault = "A";
 const::std::string kAutoReefB = "B";
 const::std::string kAutoReefC = "C";
 const::std::string kAutoReefD = "D";
 const::std::string kAutoReefE = "E";
 const::std::string kAutoReefF = "F";

};
