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

    float error_sign = left_side_drive.signum(error);

    travelled = fabs(bot.parallel_inch - initial_y_tracker_inches); // current distance travelled
    dist = fabs(dist);
    float acel_dist = fabs(acelDist(0, max_v, acel));
    if (2 * acel_dist > fabs(initial_dist))
        acel_dist =  fabs(initial_dist) / 2;

        //printf("\n");
        //printf("\nAD %.2f ", acel_dist);

    if (fabs(error) < mp_disable_length)
    {
        left_pow = drive_pid.calculate(error);
        right_pow = left_pow;
        //printf("\npd ");
    }

    else
    {
        if (travelled < acel_dist && !disable_acel) // acel portion
        {
            current_target_acel = acel * error_sign;
            pow = error_sign * sqrt(fabs(2 * acel * travelled));
            //printf("\n AP:%.2f ", pow);
            //pow = direction_multiplier * sqrt(fabs(2 * acel * travelled));
            left_pow = left_side_drive.motor_power(pow, current_target_acel, bot.left_linear_speed);
            right_pow = right_side_drive.motor_power(pow, current_target_acel, bot.right_linear_speed);
            if (fabs(left_pow) <= 1.5) left_pow = 1.5 * error_sign, right_pow = 1.5 * error_sign;
            // printf("\n acel pow: %f", pow);
        }
        else if (fabs(error) > acel_dist) // coast portion
        {
            pow = max_v * error_sign;
            left_pow = left_side_drive.motor_power(pow, 0, bot.left_linear_speed);
            right_pow = right_side_drive.motor_power(pow, 0, bot.right_linear_speed);
            // printf("\n coast pow: %f", pow);
            //printf("\ncoast ");
        }
        else
        {
            pow = error_sign * sqrt(fabs(2 * acel * error));
            current_target_acel = acel * -1 * error_sign;
            left_pow = left_side_drive.motor_power(pow, current_target_acel, bot.left_linear_speed);
            right_pow = right_side_drive.motor_power(pow, current_target_acel, bot.right_linear_speed);
            // printf("\n decel pow: %f", pow);
            //printf("\ndecel ");
        }
    }




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
    // printf("\n\nLP:%7.3f  RP:%7.3f", left_pow, right_pow);
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float hpow = 0;
    float h_error = TurnErrorFrom(bot.h_target);
    float h_error_backwards = TurnErrorFrom(bot.h_target + 180);
    // printf("\n xy_e %.3f", xy_error_length);
    // printf("     x_t %.3f y_t %.3f", bot.x_target, bot.y_target);

    if (!backwards_move)
        hpow = turn_pid.calculate(h_error);
    // printf("\n !backwards %f", hpow);
    else
    {
        hpow = turn_pid.calculate(h_error_backwards);
        // printf("\n backwards %f", hpow);
    }

    if (fabs(h_error) > classic_turn_margin && xy_error_length > turn_disable_distance && !backwards_move)
    {
        // printf("\n spot turn");
        auto_left_drive_power = hpow;
        auto_right_drive_power = -1 * hpow;
    }
    else if (fabs(h_error_backwards) > classic_turn_margin && xy_error_length > turn_disable_distance && backwards_move)
    {
        // printf("\n B spot turn");
        auto_left_drive_power = hpow;
        auto_right_drive_power = -1 * hpow;
    }
    else if (xy_error_length > turn_disable_distance)
    {
        float initial_left_pow = left_pow + hpow;
        float initial_right_pow = right_pow - hpow;
        float above_max;
        if (fabs(initial_left_pow) > 12) {
            above_max = 12 - fabs(initial_left_pow);
        }
        if (fabs(initial_right_pow) > 12) {
            float temp_above_max = 12 - fabs(initial_right_pow);
            if (temp_above_max > above_max) above_max = temp_above_max;
        }   

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
    // printf("\n (%.3f, %.3f)", auto_left_drive_power, auto_right_drive_power);
    // printf("\n (L:%.3f, R:%.3f)", auto_left_drive_power, auto_right_drive_power);
}