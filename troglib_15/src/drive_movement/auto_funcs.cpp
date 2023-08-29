#include "drive_movement/pid_movement.h"
#include "sensor_data.h"
#include "math.h"
#include "drive_movement/pid.h"
#include "motor_controller.h"
#include "drive_movement/auto_movement_loop.h"
#include "vex.h"
#include "drive_movement/auto_funcs.h"
#include "robot_config.h"
#include <iostream>
#include <iomanip>

double start_time = 0;
double end_time = 0;
void start_auto(float x, float y, float h)
{
    turn_pid.integralbound = 10;
    turn_pid.maxintegral = 2;
    drive_pid.integralbound = 5;
    drive_pid.maxintegral = 2;
    start_time = Brain.timer(vex::msec);
    enable_auto_movement = true;
    drive_controllers(1, 1);
    intake_controller(1);
    bot.reset_sensors();
    bot.setPos(x, y, h);
}

void stop_auto()
{
    end_time = Brain.timer(vex::msec) - start_time;
    delay(1000);
    enable_auto_movement = false;
    left_drive(0, 0, 0), right_drive(0, 0, 0);
    intake_at(0, 0, 0);
    delay(10000000);
}

void delay(float msec)
{
    wait(msec, vex::msec);
}

void movement_reset()
{
    drive_pid.reset(), turn_pid.reset();
    auto_left_drive_power = 0, auto_right_drive_power = 0;
}

void wait_for_turn_error(float error)
{
    while (fabs(turn_pid.error) > error)
    {
        delay(10);
    }
}

void wait_for_break_length(float error)
{
    while (bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target) > error)
    {
        delay(10);
    }
}

void turn_to(float ang, float kp, float ki, float kd, float maxSpeed, float breakang)
{
    movement_reset();
    bot.h_target = ang;
    turn_pid.setConstants(kp, ki, kd), turn_pid.maxOutput = maxSpeed;
    movement_type_index = 3;

    if (breakang > 0)
    {
        delay(10);
        wait_for_turn_error(breakang);
    }
}

void swing_on_left(float ang, float kp, float ki, float kd, float maxSpeed, float breakang)
{
    movement_reset();
    bot.h_target = ang;
    turn_pid.setConstants(kp, ki, kd), turn_pid.maxOutput = maxSpeed;
    movement_type_index = 5;

    if (breakang > 0)
    {
        delay(10);
        wait_for_turn_error(breakang);
    }
}
void swing_on_right(float ang, float kp, float ki, float kd, float maxSpeed, float breakang)
{
    movement_reset();
    bot.h_target = ang;
    turn_pid.setConstants(kp, ki, kd), turn_pid.maxOutput = maxSpeed;
    movement_type_index = 6;

    if (breakang > 0)
    {
        delay(10);
        wait_for_turn_error(breakang);
    }
}

void turn_to_xy(float x, float y, float kp, float ki, float kd, float maxSpeed, float breakang, bool backward)
{
    movement_reset();
    if (backward)
    {
        backwards_move = true;
        bot.h_target = bot.point_angle(bot.x, bot.y, x, y) - 180;
    }
    else
    {
        backwards_move = false;
        bot.h_target = bot.point_angle(bot.x, bot.y, x, y);
    }
    bot.x_target = x;
    bot.y_target = y;
    turn_pid.setConstants(kp, ki, kd), turn_pid.maxOutput = maxSpeed;
    movement_type_index = 4;
    if (breakang > 0)
    {
        delay(10);
        wait_for_turn_error(breakang);
    }
}

void swing_on_left(float x, float y, float kp, float ki, float kd, float maxSpeed, float breakang, bool backward)
{
    movement_reset();
    if (backward)
    {
        backwards_move = true;
        bot.h_target = bot.point_angle(bot.x, bot.y, x, y) - 180;
    }
    else
    {
        backwards_move = false;
        bot.h_target = bot.point_angle(bot.x, bot.y, x, y);
    }
    bot.x_target = x;
    bot.y_target = y;
    turn_pid.setConstants(kp, ki, kd), turn_pid.maxOutput = maxSpeed;
    movement_type_index = 7;
    if (breakang > 0)
    {
        delay(10);
        wait_for_turn_error(breakang);
    }
}

void swing_on_right_xy(float x, float y, float kp, float ki, float kd, float maxSpeed, float breakang, bool backward)
{
    movement_reset();
    if (backward)
    {
        backwards_move = true;
        bot.h_target = bot.point_angle(bot.x, bot.y, x, y) - 180;
    }
    else
    {
        backwards_move = false;
        bot.h_target = bot.point_angle(bot.x, bot.y, x, y);
    }
    bot.x_target = x;
    bot.y_target = y;
    turn_pid.setConstants(kp, ki, kd), turn_pid.maxOutput = maxSpeed;
    movement_type_index = 8;
    if (breakang > 0)
    {
        delay(10);
        wait_for_turn_error(breakang);
    }
}

void straight(float dist, float kp, float ki, float kd, float maxSpeed, float slew, float breakdist)
{
    movement_reset();
    pid_calc.straight_pid_length = dist;
    bot.x_target = bot.x;
    bot.y_target = bot.y;
    bot.h_target = bot.h_deg;
    pid_calc.initial_y_tracker_inches = bot.parallel_inch;
    drive_pid.slewAmount = slew;
    drive_pid.setConstants(kp, ki, kd);
    drive_pid.maxOutput = maxSpeed;
    movement_type_index = 1;
    pid_calc.straight_pid_error = (pid_calc.initial_y_tracker_inches + pid_calc.straight_pid_length) - bot.parallel_inch;
    if (breakdist > 0)
    {

        delay(30);
        while (fabs(pid_calc.straight_pid_error) > breakdist)
        {
           // printf("%.2f", fabs(pid_calc.straight_pid_error));
           // std::cout << "\n"
             //         << std::flush;
            delay(10);
        }
    }
    bot.x_target = bot.x;
    bot.y_target = bot.y;
    bot.h_target = bot.h_deg;
}

void straight_with_heading(float dist, float heading, float ykp, float yki, float ykd, float hkp, float hkd, float maxSpeed, float hmaxspeed, float slew, float breakdist)
{
    movement_reset();
    pid_calc.straight_pid_length = dist;
    bot.x_target = bot.x;
    bot.y_target = bot.y;
    bot.h_target = heading;
    pid_calc.initial_y_tracker_inches = bot.parallel_inch;
    drive_pid.slewAmount = slew;
    drive_pid.setConstants(ykp, yki, ykd);
    turn_pid.setConstants(hkp, 0, hkd);
    drive_pid.maxOutput = maxSpeed;
    turn_pid.maxOutput = hmaxspeed;
    movement_type_index = 2;
    if (breakdist > 0)
    {

        delay(30);
        while (fabs(pid_calc.straight_pid_error) > breakdist)
        {
            delay(10);
        }
    }
    bot.x_target = bot.x;
    bot.y_target = bot.y;
    bot.h_target = bot.h_deg;
}

void straight_with_heading_xy(float dist, float x, float y, float ykp, float yki, float ykd, float hkp, float hkd, float maxSpeed, float hmaxspeed, float slew, float breakdist, bool backward)
{
    movement_reset();
    pid_calc.straight_pid_length = dist;
    if (backward)
    {
        backwards_move = true;
        bot.h_target = bot.point_angle(bot.x, bot.y, x, y) - 180;
    }
    else
    {
        backwards_move = false;
        bot.h_target = bot.point_angle(bot.x, bot.y, x, y);
    }
    bot.x_target = x;
    bot.y_target = y;
    pid_calc.initial_y_tracker_inches = bot.parallel_inch;
    drive_pid.slewAmount = slew;
    drive_pid.setConstants(ykp, yki, ykd);
    turn_pid.setConstants(hkp, 0, hkd);
    drive_pid.maxOutput = maxSpeed;
    turn_pid.maxOutput = hmaxspeed;
    movement_type_index = 9;
    if (breakdist > 0)
    {

        delay(30);
        while (fabs(pid_calc.straight_pid_error) > breakdist)
        {
            delay(10);
        }
    }
    bot.x_target = bot.x;
    bot.y_target = bot.y;
    bot.h_target = bot.h_deg;
}

void classic_move_to(float x, float y, float ymax, float hmax, float ykp, float hkp, float slew, float breakLength, bool backwards)
{
    movement_reset();
    backwards_move = backwards;
    bot.x_target = x;
    bot.y_target = y;
    drive_pid.maxOutput = ymax;
    turn_pid.maxOutput = hmax;
    drive_pid.slewAmount = slew;
    drive_pid.setConstants(ykp, drive_pid.ki, drive_pid.kd);
    turn_pid.setConstants(hkp, 0, turn_pid.kd);
    movement_type_index = 10;
    if (breakLength > 0)
    {
        wait_for_break_length(breakLength);
    }
}