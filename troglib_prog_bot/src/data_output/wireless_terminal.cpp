#include "vex.h"
#include "robot_config.h"
#include "sensor_data.h"
#include "drive_movement/auto_funcs.h"
#include "drive_movement/pid_movement.h"
#include "drive_movement/sweeper.h"
#include "drive_movement/pid.h"
#include <iostream>
#include <string>
using namespace vex;

float loop_refresh_time = 100;
void standard_readout()
{
        //sweep.sweep_cycle();
    printf("\n");

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
void xy_coord_pairs()
{

   // printf("(%.3f, %.3f),(%.3f, %.3f),", bot.x, bot.y, bot.x_target, bot.y_target);
    if (counter >= 1)
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
    //wait(1000, vex::sec);
     standard_readout();

    //if(start_time > 0) xy_coord_pairs();
    std::cout << "" << std::flush;
}

int wireless_readout_thread()
{
    while (1)
    {
        print_to_terminal();

        wait(loop_refresh_time, msec);
    }
    return 0;
}