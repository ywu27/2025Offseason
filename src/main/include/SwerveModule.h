#pragma once

// FRC Libraries
#include <thread>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

// REV Libraries
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

// Random
#include "Constants.h"
#include "util/ControlUtil.h"
#include "control/PowerModule.h"

// Swerve
#include "swerve/TalonFXMotor.h"
#include "swerve/SwerveModuleState.h"
#include <frc/kinematics/SwerveModulePosition.h>

// Sensors
#include "sensors/CAN_Coder.h"

class SwerveModule
{
private:
    int steerID;
    int driveID;

    rev::spark::SparkMax steerMotor;
    rev::spark::SparkMaxConfig config;
    rev::spark::SparkClosedLoopController PIDController = steerMotor.GetClosedLoopController();

    CAN_Coder steerEnc;
    frc::PIDController steerCTR;

    float maxAccumulation = 0.4;

    // Timer for acceleration limiting
    frc::Timer aTimer = frc::Timer();

    float driveVelocitySetpoint;
    float drivePositionSetpoint;
    float steerAngleSetpoint;
    SwerveModuleState prevSetpoint = SwerveModuleState(0.0, Rotation2d(0.0));

    enum driveModeType
    {
        POSITION,
        VELOCITY
    };
    driveModeType driveMode = VELOCITY; // whether we are controlling module velocity or position
    bool moduleInhibit = true;          // True to stop the motors

    // Module Constraints
    const int maxRPMFreeSpeed = moduleMaxRPM;
    double maxVelocityAttained = 0.0;
    const float maxDriveAccelerationFPS = 25.8; // 7.603; // Feet per sec2
    const float maxDriveAccelerationRPM = 2665.993 * (25.8 / 7.6);
    const float maxSteerVelocity = 189.2; // Radians per sec

    const int maxSteerCurrent = 10;                      // Maximum current to steer motor
    const int maxDriveCurrent = swerveDriveStartCurrent; // Maximum current to steer motor
    // Must match with power module.h start value

    // TODO: Brownout module
    double currentSteerOutput = 0.0;

public:
    TalonFXMotor driveMotor;
    SwerveModule(int steerMotorID, int driveMotorID, int cancoderID);
    void initMotors();

    // Getters
    float getSteerAngleSetpoint();

    // Setpoints
    void setSteerAngleSetpoint(float setpt);

    void setDrivePositionSetpoint(float setpt);
    void setDriveVelocitySetpoint(float setpt);

    void setModuleState(SwerveModuleState setpt, bool takeShortestPath = true);
    SwerveModuleState moduleSetpointGenerator(SwerveModuleState prevSetpoint, SwerveModuleState desiredSetpoint);
    frc::SwerveModulePosition getModulePosition();

    // Encoders
    Rotation2d getSteerEncoder();
    double getSteerOutput();
    double getDriveEncoderVel();
    double getDriveEncoderPos();

    bool isFinished(float percentageBound);
    SwerveModuleState getModuleState();

    // Module state control
    void run();
    void stopModule();
    void startModule();
};