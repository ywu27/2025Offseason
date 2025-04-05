#include "Elevator.h"

void Elevator::init(){
    config.Inverted(false);
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config.closedLoop.Pid(0.04, 0, 0.0);
    config.SmartCurrentLimit(60);

    config2.Inverted(true);
    config2.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config2.closedLoop.Pid(0.04, 0, 0.0);
    config2.SmartCurrentLimit(60);

    motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    motor2.Configure(config2, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    enc.SetPosition(0);
    enc2.SetPosition(0);
}

void Elevator::setState(int state) { // 0 = start, 1 = level 1, 2 = level 2, 3 = level 3, 4 = level 4, 5 = coral station
    setpointState = levelHeight[state];

    if ((state < currentState) && (currentState != 5)) {
        double speed = mElevatorCtr.Calculate(enc.GetPosition(), zeroSetpoint + setpointState);
        speed = std::clamp(speed, -0.5, 0.5);
        elevatorCTR.SetReference(speed, rev::spark::SparkLowLevel::ControlType::kVelocity);
        elevatorCTR2.SetReference(speed, rev::spark::SparkLowLevel::ControlType::kVelocity);

        if ((enc.GetPosition() < (setpointState + 0.2)) && (enc.GetPosition() > (setpointState - 0.2))) {
            currentState = state;
        }
    } 
    else {
        elevatorCTR.SetReference(zeroSetpoint + setpointState, rev::spark::SparkLowLevel::ControlType::kPosition);
        elevatorCTR2.SetReference(zeroSetpoint + setpointState, rev::spark::SparkLowLevel::ControlType::kPosition);
    }
    currentState = state;
}

int Elevator::getCurrentState() {
    return currentState;
}

void Elevator::disable(){
    motor.StopMotor();
    motor2.StopMotor();
}

void Elevator::zero() {
    enc.SetPosition(0);
    enc2.SetPosition(0);
    zeroSetpoint = 0;
}