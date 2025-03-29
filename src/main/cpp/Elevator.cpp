#include "Elevator.h"

void Elevator::init(){
    config.Inverted(true);
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config.closedLoop.Pid(0.6, 0, 0.0);
    config.SmartCurrentLimit(20);

    config2.Inverted(true);
    config2.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config2.closedLoop.Pid(0.6, 0, 0.0);
    config2.SmartCurrentLimit(20);

    motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    motor2.Configure(config2, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    enc.SetPosition(0);
    enc2.SetPosition(0);
}

void Elevator::setState(int state, bool algae) { // 0 = start, 1 = level 1, 2 = level 2, 3 = level 3, 4 = level 4, 5 = coral station
    if (algae && ((state + 5) <= 7)) {
        setpointState = levelHeight[state + 5];
        state = state + 5;
    }
    else {
        setpointState = levelHeight[state];
    }
    elevatorCTR.SetReference(setpointState, rev::spark::SparkLowLevel::ControlType::kPosition);
    elevatorCTR2.SetReference(setpointState, rev::spark::SparkLowLevel::ControlType::kPosition);
    currentState = state;
}

int Elevator::getCurrentState() {
    return currentState;
}

void Elevator::disable(){
    motor.StopMotor();
    motor2.StopMotor();
}