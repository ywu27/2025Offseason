#pragma once

#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>
#include <array>

#define elevatorID1 9
#define elevatorID2 11

class Elevator {
    public:
        // Need to be changed, setpoints in rotations
        double startPoint = 0.0;
        double CoralLevel1 = 8.0;
        double CoralLevel2 = 22.0; 
        double CoralLevel3 = 38.0;
        double CoralLevel4 = 52.0;
        double CoralStation = 27.5;
        double AlgaeLevel1 = 8.0; 
        double AlgaeLevel2 = 15.0; 

        std::array<double, 8> levelHeight{startPoint, CoralLevel1, CoralLevel2, CoralLevel3, CoralLevel4, CoralStation, AlgaeLevel1, AlgaeLevel2};

        int currentState = 0.0;
        double setpointState;

        void init();
        void setState(int setpointState, bool algae); // 0 = start, 1 = level 1, 2 = level 2, 3 = level 3, 4 = level 4, 5 = coral station
        void runMotor(double speed);
        void disable();
        int getCurrentState();

    private:
        rev::spark::SparkMax motor = rev::spark::SparkMax(elevatorID1, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc = motor.GetEncoder();
        rev::spark::SparkMaxConfig config{};
        rev::spark::SparkClosedLoopController elevatorCTR = motor.GetClosedLoopController();

        rev::spark::SparkMax motor2 = rev::spark::SparkMax(elevatorID2, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc2 = motor2.GetEncoder();
        rev::spark::SparkMaxConfig config2{};
        rev::spark::SparkClosedLoopController elevatorCTR2 = motor2.GetClosedLoopController();
};