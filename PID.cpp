/*
PID.cpp - Library to calculate pid control effort
Created by Masoud Hassani, March 2019.
*/

#include "Arduino.h"
#include "PID.h"

PID::PID(float p, float i, float d, bool windup, float min, float max, float tol)
{
    // initialize
    gainP = p;
    gainI = i;
    gainD = d;
    controlEffort = 0;
    error = 0;
    errorPrev = 0;
    proportional = 0;
    integral = 0;
    differential = 0;
    windupGuard = windup;
    minEffort = min;
    maxEffort = max;
    tolerance = tol;
    t = micros();
}

// calculate control effort
float PID::update(float set, float current)
{
    dt = micros() - t;

    // control effort calculation
    error = set - current;
    proportional = gainP * error;
    integral += gainI * dt * error;
    differential = gainP * (error - errorPrev) / dt;

    // if error is large enough
    if (abs(error) > tolerance)
    {
        // if wind up guard is active
        if (windupGuard)
        {
            integral = min(max(integral, minEffort - proportional - differential), maxEffort - proportional - differential);
            controlEffort = integral + proportional + differential;
        }

        // if windup guard is not active
        else
        {
            controlEffort = min(max(proportional + integral + differential, minEffort), maxEffort);
        }
    }

    // if error is less than threshold, stabilize the control effort to zero
    else
    {
        controlEffort = 0.0;
    }

    // update time
    t = micros();

    // update previous error
    errorPrev = error;

    return controlEffort;
}

// setter function
void PID::setGain(float p, float i, float d)
{
    gainP = p;
    gainI = i;
    gainD = d;
}

// reset integral function
void PID::resetIntegral()
{
    integral = 0.0;
}

// reset all controls
void PID::reset()
{
    integral = 0.0;
    proportional = 0.0;
    differential = 0.0;
}
