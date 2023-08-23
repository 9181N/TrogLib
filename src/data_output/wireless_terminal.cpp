#include "vex.h"
#include "robot_config.h"
#include "sensor_data.h"
#include "drive_movement/auto_funcs.h"
#include "drive_movement/pid_movement.h"
#include <iostream>
#include <string>
using namespace vex;

float loop_refresh_time = 100;

void standard_readout()
{
    printf("tx %5.2f", bot.perpindicular_inch);
    printf(" ty %5.2f", bot.parallel_inch);
    printf(" X %6.4f", bot.x);
    printf(" Y %6.4f", bot.y);
    printf(" H %6.4f", bot.h_deg);
    printf(" Xt %6.2f", bot.x_target);
    printf(" Yt %6.2f ", bot.y_target);
    printf(" T %5.2f ", (Brain.timer(msec) - start_time) / 1000);
    printf(" ET %5.2f ", (end_time) / 1000);
}

int counter = 0;
void xy_coord_pairs()
{
    printf("(%.3f, %.3f),(%.3f, %.3f),", bot.x, bot.y, bot.x_target, bot.y_target);
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
    // wait(1000, vex::sec);
    // standard_readout();
    if(start_time > 0) xy_coord_pairs();
    std::cout << ""
              << std::flush;
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