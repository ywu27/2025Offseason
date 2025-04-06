#pragma once

#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/SmartMotionConfig.h>
#include <rev/ClosedLoopSlot.h>

#define intakeMotorID 21
#define intakeMotor2ID 20

class EndEffector {

public:
    rev::spark::SparkMax intakeMotor1 = rev::spark::SparkMax {intakeMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax intakeMotor2 = rev::spark::SparkMax {intakeMotor2ID, rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkMaxConfig intake1Config{};
    rev::spark::SparkMaxConfig intake2Config{};

    rev::spark::SparkClosedLoopController intakeCTR1 = intakeMotor1.GetClosedLoopController();
    rev::spark::SparkClosedLoopController intakeCTR2 = intakeMotor2.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder intakeEnc1 = intakeMotor1.GetEncoder();
    rev::spark::SparkRelativeEncoder intakeEnc2 = intakeMotor2.GetEncoder();

    double intakeSpeed = 10000;
    double velocity = 5;
    double coralStation = 0;
    double scoreSetpoint = 0.02;

public:
    enum EndEffectorState {
        INTAKE,
        SCORE,
        STOP
    };
    EndEffectorState currentState = STOP;

    void init();
    void disable();
    void setState(EndEffectorState state);
    void intake();
    void score();
    void scoreL4();
    void scoreL1();
    void setVelocity(double speed);
};