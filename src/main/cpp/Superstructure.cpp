#include "Superstructure.h"

void Superstructure::init() {
    mElevator.init();
    // mClimber.init();
    mEndEffector.init();
    mElevator.setState(0);
    // mClimber.position(Climber::STOW);

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
            // mClimber.disable();
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

// void Superstructure::controlClimber(int mode) { // 0 for stow / 1 for setpoint / 2 for climbing / 3 for reverse
//     mClimber.setVelocity(3000.0); // CHANGE IF NECESSARY
//     if (mode==0) {
//         mClimber.position(Climber::STOW);
//     }
//     else if (mode==1) {
//         mClimber.position(Climber::CLIMB);
//     }
//     else if (mode==2) {
//         mClimber.climb();
//     }
//     else if (mode==3) {
//         mClimber.reverse();
//     }
//     else {
//         mClimber.disable();
//     }
// }

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