#pragma once

#include "Constants.h"
#include "swerve/SwerveDriveKinematics.h"
#include "swerve/SwerveModuleState.h"
#include "SwerveSetpoint.h"

class SwerveSetpointGenerator {
private:
    SwerveDriveKinematics *mKinematics;

public:
    SwerveSetpointGenerator(SwerveDriveKinematics &kinematicModule) 
    {
        mKinematics = &kinematicModule;
    }
    void generateSetpoint(ChassisSpeeds currSetpt, SwerveSetpoint prevSetpt) 
    {
        std::vector<SwerveModuleState> states = mKinematics->toSwerveStates(currSetpt);
        states = SwerveDriveKinematics::desaturateWheelSpeeds(states, moduleMaxFPS);
    }
};