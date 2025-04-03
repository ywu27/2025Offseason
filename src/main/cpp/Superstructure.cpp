#include "Superstructure.h"

void Superstructure::init() {
    mElevator.init();
    mEndEffector.init();
    mElevator.setState(0);

    enableModules = false;
    moduleThread = std::thread(&Superstructure::periodic, this);
}

void Superstructure::periodic() {
    while (true)
    {
        if (!enableModules)
        {
            // Disable modules here
            mElevator.disable();
            mEndEffector.disable();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void Superstructure::enable() {
    enableModules = true;
}

void Superstructure::disable() {
    enableModules = false;
}

// void Superstructure::elevatorUp(bool algae) {
//     if (algae && mElevator.currentState == 6) {
//         mElevator.setState(mElevator.getCurrentState() + 1);
//     }
//     else if (mElevator.currentState < 4) {
//         mElevator.setState(mElevator.getCurrentState() + 1);
//         mEndEffector.setState(EndEffector::SCORE);
//     }
// }
// void Superstructure::elevatorDown(bool algae) {
//     if (algae && mElevator.currentState == 7) { // TRY ZERO AS WELL
//         mElevator.setState(mElevator.getCurrentState() - 1);
//     }
//     else if (mElevator.currentState > 1) {
//         mElevator.setState(mElevator.getCurrentState() - 1);
//         mEndEffector.setState(EndEffector::SCORE);
//     }
// }

void Superstructure::intakeCoral() {
    // mElevator.setState(5);
    mEndEffector.setState(EndEffector::INTAKE);
}

void Superstructure::scoreCoral() {
    mEndEffector.setState(EndEffector::SCORE);
}

double Superstructure::speedLimiter() {
    if (mElevator.currentState == 0 || mElevator.currentState == 1) {
        return 1.0;
    }
    else if (mElevator.currentState == 2 || mElevator.currentState == 3) {
        return 0.75;
    }
    else if (mElevator.currentState == 4 || mElevator.currentState == 5) {
        return 0.25;
    }
    else {
        return 1.0;
    }
}