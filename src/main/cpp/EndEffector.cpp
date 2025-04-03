#include "EndEffector.h"

void EndEffector::init() {

    intake1Config.SmartCurrentLimit(30);
    intake1Config.closedLoop.Pid(0.02, 0, 0.001);
    intake1Config.Inverted(true);
    intake1Config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    intake2Config.SmartCurrentLimit(30);
    intake2Config.closedLoop.Pid(0.02, 0, 0.001);
    intake2Config.Inverted(true);
    intake2Config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    intakeMotor1.Configure(intake1Config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    intakeMotor2.Configure(intake2Config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    intakeEnc1.SetPosition(0.0);
    intakeEnc2.SetPosition(0.0);
}

void EndEffector::disable() {
    intakeMotor1.Set(0);
    intakeMotor2.Set(0);
}

void EndEffector::setVelocity(double speed) {
    velocity = speed;
}

void EndEffector::setState(EndEffectorState state) {
    switch (state) {
    case INTAKE:
        intake();
        currentState = INTAKE;
        break;
    case SCORE:
        score();
        currentState  = SCORE;
        break;
    case STOP:
        disable();
        currentState = STOP;
        break;
    }
}

void EndEffector::intake() {
    intakeMotor1.Set(0.25);
    intakeMotor2.Set(0.25);
    if (cSensor.isTarget()) {
        disable();
    }
}

void EndEffector::score() {
    intakeCTR1.SetReference(intakeEnc1.GetPosition()+20, rev::spark::SparkLowLevel::ControlType::kPosition);
    intakeCTR2.SetReference(intakeEnc2.GetPosition()+20, rev::spark::SparkLowLevel::ControlType::kPosition);
}