#pragma once
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/DigitalInput.h>
#include <frc/I2C.h>

class ColorSensor{
    private:
        rev::ColorSensorV3 colorSensor;
        rev::ColorMatch colorMatcher;
        static constexpr frc::Color algae = frc::Color(0.1545, 0.5432, 0.3024); 
    public:
        ColorSensor(frc::I2C::Port port) : colorSensor(port){}
        frc::Color getcolor(){
             frc::Color detectedColor = colorSensor.GetColor();
             return detectedColor;

        }
        double GetProximity(){
            return colorSensor.GetProximity();
        }

        bool isTarget(){
            frc::Color detectedColor = colorSensor.GetColor();
            bool red = (detectedColor.red < 0.29) && (detectedColor.red > 0.255);
            bool green = (detectedColor.green < 0.495) && (detectedColor.green > 0.47);
            bool blue = (detectedColor.blue < 0.27) && (detectedColor.blue > 0.23);
            //bool proximity = (colorSensor.GetProximity() < 2048) && (colorSensor.GetProximity() > 500);
            if ((red && green && blue)){
                return true;
            }
            else {
                return false;
            }
        }
};