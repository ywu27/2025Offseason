#pragma once

#include "Constants.h"
#include <cmath>

class Rotation2d
{
private:
    double radians;

public:
    Rotation2d()
    {
        radians = 0.0;
    }

    Rotation2d(double angleRadians)
    {
        radians = angleRadians;
    }

    Rotation2d(double x, double y)
    {
        radians = atan2(y, x);
    }

    // Returns a new Rotation2d object that is the result of adding two rotations.
    Rotation2d operator+(const Rotation2d &other) const
    {
        double sumRadians = radians + other.radians;
        return Rotation2d(sumRadians);
    }

    // Returns a new Rotation2d object that is the inverse of the current rotation.
    Rotation2d inverse() const
    {
        double inverseRadians = -radians;
        return Rotation2d(inverseRadians);
    }

    // Converts the rotation to degrees.
    double getDegrees() const
    {
        return radians * 180.0 / PI;
    }

    double getRadians() const
    {
        return radians;
    }

    double getCos()
    {
        return cos(radians);
    }

    double getSin()
    {
        return sin(radians);
    }

    /**
     * Bound from 0 to 2pi
     */
    inline static double radiansBound(double input_radians)
    {
        double deg = (180 / PI) * input_radians;
        double output = fmod(fmod(deg, 360) + 360, 360) * (PI / 180);
        return output;
    }

    /**
     * Unimplemented
     * Polar: 0=right, positive=counterclockwise
     * Compass: 0=forward, positive=clockwise
     * Input in radians
     */
    inline static double compassToPolar(double angleRadians)
    {
        double angle = radiansBound(angleRadians);
        angle = -angle + PI_2;
        return radiansBound(angle);
    }

    /**
     * Polar: 0=right, positive=counterclockwise
     * Compass: 0=forward, positive=clockwise
     * Input in radians
     */
    inline static double polarToCompass(double angleRadians)
    {
        double angle = radiansBound(angleRadians);
        angle = -angle + PI_2;
        return radiansBound(angle);
    }

    /**
     * Bound from 0-360
     */
    inline static double degreesBound(double input_degrees)
    {
        double deg = input_degrees;
        double output = fmod(fmod(deg, 360) + 360, 360);
        return output;
    }

    inline static Rotation2d fromDegrees(double degrees)
    {
        return Rotation2d(degrees * PI / 180);
    }
};