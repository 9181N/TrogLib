#include "drive_movement/pid_movement.h"
#include "sensor_data.h"
#include "math.h"
#include "drive_movement/pid.h"
#include "motor_controller.h"
#include "drive_movement/auto_movement_loop.h"
#include "drive_movement/pathing/pps.h"
#include "vex.h"
#include <iostream>
pid_move pid_calc;

double pid_move::linear_error2d()
{
    double delta_x = bot.x_target - bot.x;                      // error on X axis
    double delta_y = bot.y_target - bot.y;                      // error on Y axis
    return delta_y * cos(bot.h_rad) + delta_x * sin(bot.h_rad); // linear error normalised for heading error
}

float initial_y_tracker_inches = 0;
float straight_pid_length = 0;
float straight_pid_error = 0;
void pid_move::straight()
{
    straight_pid_error = (initial_y_tracker_inches + straight_pid_length) - bot.parallel_inch;
    float pow = drive_pid.calculate(straight_pid_error);
    auto_left_drive_power = pow, auto_right_drive_power = pow;
}

float pid_move::turn_error_from(float targ)
{
    return bot.wrapangle(targ - bot.h_deg);
}

void pid_move::straight_heading()
{
    straight_pid_error = (initial_y_tracker_inches + straight_pid_length) - bot.parallel_inch;
    float pow = drive_pid.calculate(straight_pid_error);
    float hpow = turn_pid.calculate(turn_error_from(bot.h_target));
    auto_left_drive_power = pow + hpow, auto_right_drive_power = pow - hpow;
}

void pid_move::straight_heading_to_point()
{
    straight_pid_error = (initial_y_tracker_inches + straight_pid_length) - bot.parallel_inch;
    float pow = drive_pid.calculate(straight_pid_error);
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float hpow = 0;
    if (!backwards_move)
        pow = turn_pid.calculate(turn_error_from(bot.h_target));
    if (backwards_move)
        pow = turn_pid.calculate(turn_error_from(bot.h_target + 180));
    auto_left_drive_power = pow + hpow, auto_right_drive_power = pow - hpow;
}

void pid_move::turn()
{

    auto_left_drive_stopping = 0, auto_right_drive_stopping = 0;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    float pow = turn_pid.calculate(turn_error_from(bot.h_target));
    if (auto_wrap_turn_target)
        pow = turn_pid.calculate(bot.h_target - bot.h_deg);
    auto_left_drive_power = pow, auto_right_drive_power = -1 * pow;
}

void pid_move::turn_to_xy()
{
    auto_left_drive_stopping = 0, auto_right_drive_stopping = 0;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float pow = 0;
    if (!backwards_move)
        pow = turn_pid.calculate(turn_error_from(bot.h_target));
    if (backwards_move)
        pow = turn_pid.calculate(turn_error_from(bot.h_target + 180));

    auto_left_drive_power = pow, auto_right_drive_power = -1 * pow;
}

void pid_move::swing_turn_on_left()
{
    auto_left_drive_stopping = 2, auto_right_drive_stopping = 0;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    float pow = turn_pid.calculate(turn_error_from(bot.h_target));
    auto_left_drive_power = 0, auto_right_drive_power = -1 * pow;
}

void pid_move::swing_turn_on_right()
{
    auto_left_drive_stopping = 0, auto_right_drive_stopping = 2;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    float pow = turn_pid.calculate(turn_error_from(bot.h_target));
    auto_left_drive_power = pow, auto_right_drive_power = 0;
}

void pid_move::swing_turn_to_xy_on_left()
{
    auto_left_drive_stopping = 2, auto_right_drive_stopping = 0;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float pow = 0;
    if (!backwards_move)
        pow = turn_pid.calculate(turn_error_from(bot.h_target));
    if (backwards_move)
        pow = turn_pid.calculate(turn_error_from(bot.h_target + 180));
    auto_left_drive_power = 0, auto_right_drive_power = -1 * pow;
}

void pid_move::swing_turn_to_xy_on_right()
{
    auto_left_drive_stopping = 0, auto_right_drive_stopping = 2;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float pow = 0;
    if (!backwards_move)
        pow = turn_pid.calculate(turn_error_from(bot.h_target));
    if (backwards_move)
        pow = turn_pid.calculate(turn_error_from(bot.h_target + 180));
    auto_left_drive_power = pow, auto_right_drive_power = 0;
}

float turn_disable_distance = 5;
float classic_turn_margin = 80;
bool first;
void pid_move::classic_to_point()
{
    auto_left_drive_stopping = 0, auto_right_drive_stopping = 0;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float xy_error_length = bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target);
    float hpow = 0;
    if (!backwards_move)
        hpow = turn_pid.calculate(turn_error_from(bot.h_target));
    else
    {
        hpow = turn_pid.calculate(turn_error_from(bot.h_target + 180));
    }

    float ypow = drive_pid.calculate(linear_error2d());
    // printf("(%.2f,%.2f)\n",ypow, hpow);
    float relative_angle = 0;
    if (backwards_move)
    {
        relative_angle = fabs(bot.relativeangle(bot.x, bot.y, bot.x_target, bot.y_target)) - 180;
    }
    else
    {
        relative_angle = fabs(bot.relativeangle(bot.x, bot.y, bot.x_target, bot.y_target));
    }

    if (relative_angle > classic_turn_margin && xy_error_length > 5)
    {
        // printf("here: %.2f\n", relative_angle);
        auto_left_drive_power = hpow;
        auto_right_drive_power = -1 * hpow;
    }
    else if (xy_error_length > turn_disable_distance)
    {
        // printf("here1:\n");
        auto_left_drive_power = ypow + hpow,
        auto_right_drive_power = ypow - hpow;
        first = true;
    }
    else if (first == 1)
    {
        // printf("here2:\n");
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

        auto_left_drive_power = ypow, auto_right_drive_power = ypow;

        first = false;
    }
    else
    {
        // printf("here3:\n");
        auto_left_drive_power = ypow, auto_right_drive_power = ypow;
    }

    /*  printf("\n\n");
  printf("LE %5.2f", linear_error2d());
  printf("\n");
  printf("P %5.2f", turn_disable_distance);
*/
}

void pid_move::classic_pps_path()
{
    get_line.ppsFollowPath();

    auto_left_drive_stopping = 0, auto_right_drive_stopping = 0;
    auto_left_drive_vel = false, auto_right_drive_vel = false;
    bot.h_target = bot.point_angle(bot.x, bot.y, bot.x_target, bot.y_target);
    float xy_error_length = bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target);
    // printf("xy: %.2f (%.2f, %.2f) \n", xy_error_length, path1.x[path1.fidelity], path1.y[path1.fidelity]);
    float hpow = 0;
    float turn_error = turn_error_from(bot.h_target);
    float b_turn_error = turn_error_from(bot.h_target + 180);
    float relative_angle = 0;

    if (!backwards_move)
    {
        hpow = turn_pid.calculate(turn_error);
        relative_angle = fabs(bot.wrapangle(turn_error));
    }
    else
    {
        hpow = turn_pid.calculate(b_turn_error);
        relative_angle = fabs(bot.wrapangle(b_turn_error));
    }

    float ypow = drive_pid.calculate(linear_error2d());
    if (relative_angle > classic_turn_margin && xy_error_length > 5)
    {
        printf("spot: %.2f, %.2f\n\n", relative_angle, classic_turn_margin);
        auto_left_drive_power = hpow;
        auto_right_drive_power = -1 * hpow;
    }
    else if (xy_error_length > turn_disable_distance)
    {
        auto_left_drive_power = ypow + hpow,
        auto_right_drive_power = ypow - hpow;
        first = true;
    }
    else if (first == 1)
    {
        printf("first\n\n");
        if (backwards_move)
        {
            path1.x[path1.fidelity] = bot.x + bot.vector_x_length_at_theta(bot.h_deg + 180, xy_error_length);
            path1.y[path1.fidelity] = bot.y + bot.vector_y_length_at_theta(bot.h_deg + 180, xy_error_length);
            bot.x_target = path1.x[path1.fidelity];
            bot.y_target = path1.y[path1.fidelity];
        }
        if (!backwards_move)
        {
            path1.x[path1.fidelity] = bot.x + bot.vector_x_length_at_theta(bot.h_deg, xy_error_length);
            path1.y[path1.fidelity] = bot.y + bot.vector_y_length_at_theta(bot.h_deg, xy_error_length);
            bot.x_target = path1.x[path1.fidelity];
            bot.y_target = path1.y[path1.fidelity];
        }

        auto_left_drive_power = ypow, auto_right_drive_power = ypow;

        first = false;
    }
    else
    {
        auto_left_drive_power = ypow, auto_right_drive_power = ypow;
    }
}
