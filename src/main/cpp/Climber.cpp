#include "Climber.h"

void Climber::init(){
    config.Inverted(false);
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config.closedLoop.Pid(0.2, 0, 0.1);

    config.SmartCurrentLimit(20);
    motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

void Climber::disable() {
    motor.Set(0);
}

void Climber::climb() {
    motor.Set(1);
}

void Climber::reverse() {
    motor.Set(-1);
}

void Climber::position(climberState climbState) {
    switch (climbState) {
        case (STOW):
            climberCTR.SetReference(0, rev::spark::SparkLowLevel::ControlType::kPosition);
        case (CLIMB):
            climberCTR.SetReference(145, rev::spark::SparkLowLevel::ControlType::kPosition);
    }
}