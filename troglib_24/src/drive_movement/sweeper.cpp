#include "drive_movement/pid_movement.h"
#include "sensor_data.h"
#include "math.h"
#include "drive_movement/pid.h"
#include "motor_controller.h"
#include "drive_movement/auto_movement_loop.h"
#include "vex.h"
#include "drive_movement/sweeper.h"
#include "robot_config.h"
#include <iostream>


float sweeper_out_position = 80;
float sweeper_in_position = 260;

void sweeper_class::wait_until_error(float margin) {
    wait(50, vex::msec);
    while(fabs(error) > margin) {
       // printf("\n E:%.2f", error);
        wait(10, vex::msec);
    }
}



void sweeper_class::move_sweeper(bool out) {
    if (out) sweeper_pid_targ = sweeper_out_position;
    else sweeper_pid_targ = sweeper_in_position;
}

void sweeper_class::sweep_cycle() {
 sweeper_position = sweeper_pot.value(vex::deg);
error = sweeper_pid_targ - sweeper_position;
float pow = sweeper_pid.calculate(error);
sweeper_pid.maxOutput = 12;
sweep_at(pow, 0);
}

int sweeper_thread()
{
    while (1)
    {
        sweep.sweep_cycle();
        wait(10, vex::msec);
    }
    return 0;
}