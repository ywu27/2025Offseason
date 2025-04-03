#pragma once

#include <thread>
#include "Elevator.h"
#include "EndEffector.h"

class Superstructure 
{
private:
    std::thread moduleThread;
    bool enableModules;

public:
    Elevator mElevator;
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
    double speedLimiter();
};