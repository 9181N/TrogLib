#include "drive_movement/pid_movement.h"
#include "sensor_data.h"
#include "math.h"
#include "drive_movement/pid.h"
#include "drive_movement/motion_profile.h"
#include "motor_controller.h"
#include "drive_movement/auto_movement_loop.h"
#include "vex.h"

MP_move mp_calc;

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
    // d = (vf^2-vi^2)/2a
    float vf_sqd = final_v * final_v;
    float vi_sqd = initial_v * initial_v;
    float dist = (vf_sqd - vi_sqd) / (2 * a);
    return dist;
}

float MP_move::mp_1d_speed(float dist, float max_v, float a)
{
    float target_dist = linearError2D();
    float decel_dist = acelDist(max_v, 0, a);
    if (target_dist < decel_dist)
    {
        // decel
    }
    else
    {
        // acel
    }
    float pow = 0;
    return pow;
}

void MP_move::classicToPoint()
{
    auto_left_drive_stopping = 0, auto_right_drive_stopping = 0;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float hpow = 0;
    if (!backwards_move)
        hpow = turn_pid.calculate(TurnErrorFrom(bot.h_target));
    else
    {
        hpow = turn_pid.calculate(TurnErrorFrom(bot.h_target + 180));
    }

    float ypow = drive_pid.calculate(linearError2D());
    if (fabs(bot.relativeangle(bot.x, bot.y, bot.x_target, bot.y_target)) > classic_turn_margin && linearError2D() > 5)
    {
        auto_left_drive_power = hpow;
        auto_right_drive_power = -1 * hpow;
    }
    else if (fabs(linearError2D()) > turn_disable_distance)
    {
        auto_left_drive_power = ypow + hpow,
        auto_right_drive_power = ypow - hpow;
        first = true;
    }
    else if (first == 1)
    {
        if (backwards_move)
        {
            bot.x_target = bot.x + bot.vector_x_length_at_theta(bot.h_deg + 180, bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target));
            bot.y_target = bot.y + bot.vector_y_length_at_theta(bot.h_deg + 180, bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target));
        }
        if (!backwards_move)
        {
            bot.x_target = bot.x + bot.vector_x_length_at_theta(bot.h_deg, bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target));
            bot.y_target = bot.y + bot.vector_y_length_at_theta(bot.h_deg, bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target));
        }

        auto_left_drive_power = ypow, auto_right_drive_power = ypow;

        first = false;
    }
    else
    {
        auto_left_drive_power = ypow, auto_right_drive_power = ypow;
    }

    /*  printf("\n\n");
  printf("LE %5.2f", linear_error2d());
  printf("\n");
  printf("P %5.2f", turn_disable_distance);
*/
}
