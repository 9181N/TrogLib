#include "vex.h"
#include "robot_config.h"
#include "sensor_data.h"
#include "drive_movement/auto_funcs.h"
#include "drive_movement/pid_movement.h"
#include "drive_movement/sweeper.h"
#include "drive_movement/pid.h"
#include "drive_movement/motion_profile.h"
#include "drive_movement/auto_movement_loop.h"
#include "drive_movement/motion_profile.h"

#include <iostream>
#include <string>
using namespace vex;
bool wireless_terminal_on = true;
float loop_refresh_time = 20;
void standardReadout()
{
    // sweep.sweep_cycle();
    printf("\n");
    printf("v:%.2f ", bot.linear_speed);
    printf("tx%.2f ", bot.perpindicular_inch);
    printf("ty%.2f ", bot.parallel_inch);
    printf("X:%.3f ", bot.x);
    printf("Y:%.3f ", bot.y);
    printf("H:%.3f ", bot.h_deg);
    printf("xt%.2f ", bot.x_target);
    printf("yt%.2f ", bot.y_target);
    printf("T%.2f ", (Brain.timer(msec) - start_time) / 1000);
    printf("ET%.2f ", (end_time) / 1000);
}

int counter = 0;
void graphingFormat()
{
    printf("(%.3f, %.3f),", float(Brain.timer(msec) - start_time) / 1000, mp_calc.linearError2D());
    //printf("(%.3f, %.3f),", float(Brain.timer(msec) - start_time) / 1000, bot.linear_speed);

    //printf("(%.3f, %.3f),", float(Brain.timer(msec) - start_time) / 1000, mp_calc.error);
    //printf("(%.3f, %.3f),(%.3f, %.3f),", bot.x, bot.y, bot.x_target, bot.y_target);
    if (counter >= 2)
    {
        printf("\n");
        counter = 0;
    }
    else
    {
        counter++;
    }
}
void print_to_terminal()
{
    if (wireless_terminal_on)
    {
        //standardReadout();

        // wait(1000, vex::sec);
         //graphingFormat();
        //if(start_time > 0 && end_time == 0) graphingFormat();
    }
    std::cout << "" << std::flush;
}

int wireless_readout_thread()
{
    printf("\n");
    printf("\n");
    while (1)
    {
        print_to_terminal();

        wait(loop_refresh_time, msec);
    }
    return 0;
}