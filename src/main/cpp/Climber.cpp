#include "Climber.h"

void Climber::init(){
    config.Inverted(false);
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config.closedLoop.Pid(0.2, 0.0005, 0.0);
    
    //This value will be changed based on testing
    config.SmartCurrentLimit(20);
    motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    enc.SetPosition(0);
}

void Climber::disable() {
    motor.StopMotor();
}

void Climber::setVelocity(double speed){
    velocity = speed;
}

void Climber::climb() {
    //climberCTR.SetReference(velocity, rev::spark::SparkBase::ControlType::kVelocity);
    motor.Set(1); // TRY THIS AS WELL
}

void Climber::reverse() {
    motor.Set(-1);
}

void Climber::position(climberState climbState) {
    switch (climbState) {
        case (STOW):
            climberCTR.SetReference(0, rev::spark::SparkLowLevel::ControlType::kPosition);
        case (CLIMB):
            climberCTR.SetReference(climbSetpoint, rev::spark::SparkLowLevel::ControlType::kPosition);
    }
}