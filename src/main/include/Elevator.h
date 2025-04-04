#pragma once

#include <frc/DigitalInput.h>
#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>
#include <array>

//UPDATE HERE, conflicting CAN IDs
#define elevatorID1 9
#define elevatorID2 19

class Elevator {
    public:
        // Need to be changed, setpoints in rotations
        double startPoint = 0.0;
        double CoralLevel1 = 34.7854;
        double CoralLevel2 = 49.786; 
        double CoralLevel3 = 73.502;
        double CoralLevel4 = 110;
        double CoralStation = 5.386;

        std::array<double, 6> levelHeight{startPoint, CoralLevel1, CoralLevel2, CoralLevel3, CoralLevel4, CoralStation};

        int currentState;
        double setpointState;

        void init();
        void setState(int setpointState); // 0 = start, 1 = level 1, 2 = level 2, 3 = level 3, 4 = level 4, 5 = coral station
        void runMotor(double speed);
        void disable();
        int getCurrentState();
        void zero();

        rev::spark::SparkMax motor = rev::spark::SparkMax(elevatorID1, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc = motor.GetEncoder();
        rev::spark::SparkMaxConfig config{};
        rev::spark::SparkClosedLoopController elevatorCTR = motor.GetClosedLoopController();

        rev::spark::SparkMax motor2 = rev::spark::SparkMax(elevatorID2, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc2 = motor2.GetEncoder();
        rev::spark::SparkMaxConfig config2{};
        rev::spark::SparkClosedLoopController elevatorCTR2 = motor2.GetClosedLoopController();

        frc::DigitalInput limitSwitch{0};
};