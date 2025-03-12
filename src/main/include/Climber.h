#pragma once

#include <rev/SparkMax.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>

#define climberID 13

class Climber{
    private:
        rev::spark::SparkMax motor = rev::spark::SparkMax(climberID, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc = motor.GetEncoder();
        rev::spark::SparkMaxConfig config{};
        rev::spark::SparkClosedLoopController climberCTR = motor.GetClosedLoopController();

    public:
        enum climberState {
            STOW,
            CLIMB
        };

        void init();
        void disable();
        void setVelocity(double speed);
        void climb();
        void reverse();
        void position(climberState climbState);
};