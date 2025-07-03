#include "DeAlgae.h"

void DeAlgae::init() {

    algae1Config.SmartCurrentLimit(20);
    algae1Config.closedLoop.Pid(0.00020, 0, 0.001);
    algae1Config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    algae2Config.SmartCurrentLimit(20);
    algae2Config.closedLoop.Pid(0.00020, 0, 0.001);
    algae2Config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    algaeMotor1.Configure(algae1Config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    algaeMotor2.Configure(algae2Config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    algaeEnc1.SetPosition(0.0);
    algaeEnc2.SetPosition(0.0);
}

void DeAlgae::disable() {
    algaeMotor1.StopMotor();
    algaeMotor2.StopMotor();
}

void DeAlgae::setVelocity(double speed) {
    algaeSpeed = speed;
}

void DeAlgae::setState(int armState) {
    if(armState == 0) {
        currentArmState = 0;
        algaeMotor1.Set(0.1);
    } else if(armState==1) {
        currentArmState = 1;
        algaeCTR1.SetReference(upSetpoint, rev::spark::SparkLowLevel::ControlType::kPosition);
        algaeMotor1.Set(-0.1);
    }
    else{
        algaeMotor1.Set(0);
    }
}

void DeAlgae::intake() {
    algaeMotor2.Set(0.1);
}

void DeAlgae::outtake() {
    algaeMotor2.Set(-0.1);
}