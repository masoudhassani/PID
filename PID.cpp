/*
PID.cpp - Library to calculate pid control effort
Created by Masoud Hassani, March 2019.
*/

#include "Arduino.h"
#include "PID.h"

PID::PID(float p, float i, float d, bool windup, float thresh)
{
    // initialize
    gainP = p;
    gainI = i;
    gainD = d;
    controlEffort = 0;
    error = 0;
    proportional = 0;
    integral = 0;
    differential = 0;
    windupGuard = windup;
    t = micros();
}

// calculate control effort
float PID::update(float set, float current)
{
    dt = micros() - t;

    // control effort calculation
    error = set - current;
    proportional = gainP * error;
    integral += gainI * dt;
    differential = gainP;
    controlEffort = proportional + integral + differential;

    // update time
    t = micros();

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
    integral = 0;
}
