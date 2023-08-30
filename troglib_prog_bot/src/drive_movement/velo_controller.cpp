#include "drive_movement/velo_controller.h"

float velo_controller::signum(float input)
{
    float output;
    if (input == 0.0)
        output = 0.0;
    if (input > 0.0)
        output = 1.0;
    if (input < 0.0)
        output = -1.0;
    return output;
}

float velo_controller::motor_power(float target_speed, float target_acceleration, float current_speed)
{
    float acceleration_feedforward;
    float acel_sign = signum(target_acceleration);
    float speed_sign = signum(target_speed);
    if (acel_sign == speed_sign)
    {
        acceleration_feedforward = Ka * target_acceleration;
    }
    else
    {
        acceleration_feedforward = Kd * target_acceleration;
    }
    if (target_acceleration == 0) acceleration_feedforward = 0;
    current_speed = current_speed / 2;
    float error = target_speed - current_speed;
    float output = (Kv * target_speed) + (acceleration_feedforward) + (error * Kp);
    return output;
}