#include "math.h"
#include "mkhlib/pid.h"

using namespace mkhlib;

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
    cur = 0, targ = 0, error = 0, last_error = 0, delta_error = 0, sum_error = 0, slew_amount = 0, prev_output = 0;
}

void PID::setIntegral(float cap, float bound)
{
    max_integral = cap, integral_bound = bound;
}

float PID::calculate(float inputError)
{ // PID calculation using the input as the error
    error = inputError;
    delta_error = error - last_error;
    if (signum(delta_error) == signum(error))
        delta_error = 0; // if sign of delaerror = sign error deltaeror = 0
    if (fabs(error) < integral_bound)
        sum_error += error;
    if (signum(error) != signum(last_error))
        sum_error = 0; // if sign error != sign lasterror sumerror = 0
    if (fabs(sum_error * ki) > max_integral)
        sum_error = (max_integral * signum(sum_error)) / ki;
    output = (error * kp) + (sum_error * ki) + (delta_error * kd); // calculates PID output
    last_error = error;

    if (slew_amount > 0)
    { // calculate slew if slew is enabled
        if (output > 0)
        {
            if (output > (prev_output + slew_amount))
            {
                output = prev_output + slew_amount;
            }
        }
        if (output < 0)
        {
            if (output < (prev_output - slew_amount))
            {
                output = prev_output - slew_amount;
            }
        }
    }

    if (fabs(output) > max_output)
    { // cap the output to the max allowed
        output = max_output * signum(output);
    }
    prev_output = output;
    return output;
}