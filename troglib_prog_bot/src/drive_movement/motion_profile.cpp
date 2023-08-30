#include "drive_movement/pid_movement.h"
#include "sensor_data.h"
#include "math.h"
#include "drive_movement/pid.h"
#include "drive_movement/motion_profile.h"
#include "motor_controller.h"
#include "drive_movement/auto_movement_loop.h"
#include "vex.h"
#include "drive_movement/velo_controller.h"

double MP_move::linearError2D()
{
    double delta_x = bot.x_target - bot.x;                      // error on X axis
    double delta_y = bot.y_target - bot.y;                      // error on Y axis
    return delta_y * cos(bot.h_rad) + delta_x * sin(bot.h_rad); // linear error normalised for heading error
}

float MP_move::TurnErrorFrom(float targ)
{
    return bot.wrapangle(targ - bot.h_deg);
}

float MP_move::acelDist(float initial_v, float final_v, float a)
{
    // vf^2 = vi^2 + 2ad
    // vi^2 = vf^2 - 2ad   vf = 0 so vi =  -1 * sqrt(fabs(2ad)
    // d = (vf^2-vi^2)/2a
    float vf_sqd = final_v * final_v;
    float vi_sqd = initial_v * initial_v;
    float dist = (vf_sqd - vi_sqd) / (2 * a);
    return fabs(dist);
}

float MP_move::mp_1d_speed(float dist, float max_v, float acel, bool adaptive)
{
    if (adaptive) error = linearError2D();
    else error = (initial_y_tracker_inches + dist) - bot.parallel_inch;
    travelled = fabs(bot.parallel_inch - initial_y_tracker_inches); // current distance travelled
    dist = fabs(dist);
    float acel_dist = acelDist(max_v, 0, acel);
    if (2 * acel_dist > dist)
    acel_dist = dist/2;


    if (fabs(error) < mp_disable_length)
    {
        left_pow = drive_pid.calculate(error);
        right_pow = left_pow;
    }

    else
    {
        if (travelled < acel_dist) // acel portion
        {
            current_target_acel = acel * direction_multiplier;
            pow = direction_multiplier * sqrt(fabs(2 * acel * travelled));
            //printf("\n acel pow: %f", pow);
        }
        else if (fabs(error) > acel_dist) // coast portion
        {
            pow = max_v * direction_multiplier;
            current_target_acel = 0;
            //printf("\n coast pow: %f", pow);
        }
        else
        {
            pow = direction_multiplier * sqrt(fabs(2 * acel * error));
            current_target_acel = acel * -1 * direction_multiplier;
            //printf("\n decel pow: %f", pow);
        }

        left_pow = left_side_drive.motor_power(pow, current_target_acel, bot.left_linear_speed);
        right_pow = right_side_drive.motor_power(pow, current_target_acel, bot.right_linear_speed);
    }
    if (fabs(error) > mp_disable_length && fabs(left_pow) <= 1.5)
        left_pow = 1.5 * direction_multiplier, right_pow = 1.5 * direction_multiplier;

    output = pow;
    return pow;
}

void MP_move::straight()
{
    auto_left_drive_stopping = 1, auto_right_drive_stopping = 1;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    mp_1d_speed(dist, max_speed, acel, false);
    auto_left_drive_power = left_pow, auto_right_drive_power = right_pow;
}

void MP_move::classicToPoint()
{
    auto_left_drive_stopping = 1, auto_right_drive_stopping = 1;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    float xy_error_length = bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target);
    mp_1d_speed(dist, max_speed, acel, true);
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float hpow = 0;
    if (!backwards_move)
        hpow = turn_pid.calculate(TurnErrorFrom(bot.h_target));
    else
    {
        hpow = turn_pid.calculate(TurnErrorFrom(bot.h_target + 180));
    }
        //printf("\n (%.3f, %.3f)", bot.x_target, bot.y_target);

    if (fabs(bot.relativeangle(bot.x, bot.y, bot.x_target, bot.y_target)) > classic_turn_margin && xy_error_length > turn_disable_distance)
    {
        auto_left_drive_power = hpow;
        auto_right_drive_power = -1 * hpow;
    }
    else if (xy_error_length > turn_disable_distance)
    {

        auto_left_drive_power = left_pow + hpow,
        auto_right_drive_power = right_pow - hpow;
        first = true;
    }
    else if (first == 1)
    {
        if (backwards_move)
        {
            bot.x_target = bot.x + bot.vector_x_length_at_theta(bot.h_deg + 180, xy_error_length);
            bot.y_target = bot.y + bot.vector_y_length_at_theta(bot.h_deg + 180, xy_error_length);
        }
        if (!backwards_move)
        {
            bot.x_target = bot.x + bot.vector_x_length_at_theta(bot.h_deg, xy_error_length);
            bot.y_target = bot.y + bot.vector_y_length_at_theta(bot.h_deg, xy_error_length);
        }

        auto_left_drive_power = left_pow, auto_right_drive_power = right_pow;

        first = false;
    }
    else
    {
        auto_left_drive_power = left_pow, auto_right_drive_power = right_pow;
    }
        printf("\n (%.3f, %.3f)", auto_left_drive_power, auto_right_drive_power);
}