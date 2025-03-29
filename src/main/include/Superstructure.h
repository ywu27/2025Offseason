#pragma once

#include <thread>
#include "Climber.h"
#include "Elevator.h"
#include "EndEffector.h"

class Superstructure 
{
private:
    std::thread moduleThread;
    bool enableModules;

public:
    Elevator mElevator;
    Climber mClimber;
    EndEffector mEndEffector;

    void init();
    void periodic();
    void enable();
    void disable();
    void intakeCoral();
    void controlIntake(int mode);
    void elevatorUp(bool algae);
    void elevatorDown(bool algae);
    void scoreCoral();
    void controlClimber(int position);
    double speedLimiter();
};