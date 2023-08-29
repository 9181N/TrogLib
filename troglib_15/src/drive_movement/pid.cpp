#include "math.h"
#include "drive_movement/pid.h"

float PID::signum(float input)
{
    if (input > 0)
        return 1;
    else if (input < 0)
        return -1;
    else { 
        return 0;
    }
}

void PID::setConstants(float KP, float KI, float KD)
{
    kp = KP;
    ki = KI;
    kd = KD;
}

void PID::reset()
{
    cur = 0, targ = 0, error = 0, lasterror = 0, deltaerror = 0, sumerror = 0, slewAmount = 0, prevOutput = 0;
}

void PID::setIntegral(float cap, float bound)
{
    maxintegral = cap, integralbound = bound;
}

float PID::calculate(float inputError)
{ // PID calculation using the input as the error
    error = inputError;
    deltaerror = error - lasterror;
    if (signum(deltaerror) == signum(error))
        deltaerror = 0; // if sign of delaerror = sign error deltaeror = 0
    if (fabs(error) < integralbound)
        sumerror += error;
    if (signum(error) != signum(lasterror))
        sumerror = 0; // if sign error != sign lasterror sumerror = 0
    if (fabs(sumerror * ki) > maxintegral)
        sumerror = (maxintegral * signum(sumerror)) / ki;
    output = (error * kp) + (sumerror * ki) + (deltaerror * kd); // calculates PID output
    lasterror = error;

    if (slewAmount > 0)
    { // calculate slew if slew is enabled
        if (output > 0)
        {
            if (output > (prevOutput + slewAmount))
            {
                output = prevOutput + slewAmount;
            }
        }
        if (output < 0)
        {
            if (output < (prevOutput - slewAmount))
            {
                output = prevOutput - slewAmount;
            }
        }
    }

    if (fabs(output) > maxOutput)
    { // cap the output to the max allowed
        output = maxOutput * signum(output);
    }
    prevOutput = output;
    return output;
}