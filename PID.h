/*
PID.h - Library to calculate pid control effort
Created by Masoud Hassani, March 2019.
*/
#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID
{
    private:
        unsigned long   dt;   // delta time for pid calculation
        unsigned long   t;    // variable to hold old time in micro seconds
        bool  windupGuard;  // if true, windup guard is active
        float gainP; // p gain
        float gainI; // i gain
        float gainD; // d gain
        float controlEffort;  // pid control effort
        float error;  // setpoint - current
        float errorPrev;
        float integral;
        float proportional;
        float differential;
        float minEffort;
        float maxEffort;
        float tolerance;
        float maxIntegral = 0.2;

    public:
        PID(float p, float i, float d, bool windup, float min, float max, float tol);
        void  setGain(float p, float i, float d);    // sets gains
        void  resetIntegral();    // reset integral
        void  reset();  // reset all controls
        void  setEffort(float effMax, float effMin);  // update min/max effort
        void  constraintIntegral(float maxInt);   // set windup guard value of inegral constraint
        float update(float set, float current);  // main callback to calculate control effort
        float* returnControlEfforts();
};

#endif
