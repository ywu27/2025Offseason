#pragma once

#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/SmartMotionConfig.h>
#include <rev/ClosedLoopSlot.h>


#define algaeMotorID 13
#define algaeMotor2ID 3

class DeAlgae {

public:
    rev::spark::SparkMax algaeMotor1 = rev::spark::SparkMax {algaeMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax algaeMotor2 = rev::spark::SparkMax {algaeMotor2ID, rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkMaxConfig algae1Config{};
    rev::spark::SparkMaxConfig algae2Config{};

    rev::spark::SparkClosedLoopController algaeCTR1 = algaeMotor1.GetClosedLoopController();
    rev::spark::SparkClosedLoopController algaeCTR2 = algaeMotor2.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder algaeEnc1 = algaeMotor1.GetEncoder();
    rev::spark::SparkRelativeEncoder algaeEnc2 = algaeMotor2.GetEncoder();

    double algaeSpeed = 10000;
    double downSetpoint = 0;
    double upSetpoint = 0.02;

    

public:
    int currentArmState = 0;//0 is down, 1 is up
    int currentAlgaefyerState = 0;//0 is algae, 1 is outtake
    int movement = 0;//update this based on how much encoder val needs to change to outtake
    void init();
    void disable();
    void setState(int armState);
    void intake();
    void outtake();
    void setVelocity(double speed);
};