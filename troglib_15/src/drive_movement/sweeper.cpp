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


void sweeper_class::sweep_cycle() {
 sweeper_position = sweeper_pot.value(vex::deg);
float error = sweeper_pid_targ - sweeper_position;
float pow = sweeper_pid.calculate(error);
sweeper_pid.maxOutput = 12;
sweep_at(pow, 0);
}