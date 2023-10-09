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
float loop_refresh_time = 100;
void standardReadout()
{
    // sweep.sweep_cycle();
    printf("\n");
    //printf("v:%.2f ", bot.linear_speed);
    //printf("tx%.2f ", bot.perpindicular_inch);
    //printf("ty%.2f ", bot.parallel_inch);
    //printf("T:%7.3f  ", mp_calc.travelled);
    //printf("E:%7.3f  ", mp_calc.error);
    printf("X:%7.3f  ", bot.x);
    printf("Y:%7.3f  ", bot.y);
    printf("H:%7.3f  ", bot.h_deg);
    printf("xt%7.2f  ", bot.x_target);
    printf("yt%7.2f  ", bot.y_target);
    printf("ht%7.2f  ", bot.h_target);
    printf("L%7.2f  ", bot.point_distance(bot.x, bot.y, bot.x_target, bot.y_target));

    //printf("T%7.2f ", (Brain.timer(msec) - start_time) / 1000);
    printf("ET%7.2f ", (end_time) / 1000);
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
        standardReadout();
        //printf("\nE:%.2f, Y%.2f", mp_calc.error, bot.y);
        //printf("   LP:%7.3f  ", mp_calc.left_pow);
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