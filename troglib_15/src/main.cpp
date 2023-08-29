#include "vex.h"
#include "motor_controller.h"
#include "drive_movement/user_control.h"
#include "data_output/brain_screen.h"
#include "data_output/wireless_terminal.h"
#include "sensor_data.h"
#include "drive_movement/pid.h"
#include "drive_movement/auto_movement_loop.h"
#include "autos.h"
#include "drive_movement/sweeper.h"

#include "mkhlib/datatypes.hpp"

using namespace vex;
brain Brain;
controller Controller;
competition Competition;

//15 inch

motor intake = motor(PORT11, ratio6_1, true);

motor LS_bottom = motor(PORT9, ratio18_1, true);
motor LS_middle = motor(PORT8, ratio18_1, false);
motor LS_top = motor(PORT7, ratio18_1, true);
motor LS_front = motor(PORT10, ratio18_1, true);

motor RS_bottom = motor(PORT2, ratio18_1, false);
motor RS_middle = motor(PORT3, ratio18_1, true);
motor RS_top = motor(PORT4, ratio18_1, false);
motor RS_front = motor(PORT1, ratio18_1, false);

motor_group left_drive_motors = motor_group(LS_bottom, LS_middle, LS_top, LS_front);
motor_group right_drive_motors = motor_group(RS_bottom, RS_middle, RS_top, RS_front);

optical color_sensor = optical(PORT13);
inertial IMU1 = inertial(PORT12);
encoder TY = encoder(Brain.ThreeWirePort.C);
encoder TX = encoder(Brain.ThreeWirePort.A);
potV2 sweeper_pot = potV2(Brain.ThreeWirePort.H);
motor sweeper = motor(PORT14, ratio36_1, false);


//
/*
//24 inch
motor intake = motor(PORT20, ratio6_1, true);
motor LS_front = motor(PORT19, ratio18_1, false);
motor LS_back = motor(PORT10, ratio18_1, false);
motor RS_front = motor(PORT12, ratio18_1, true);
motor RS_back = motor(PORT1, ratio18_1, true);

motor_group left_drive_motors = motor_group(LS_front, LS_back);
motor_group right_drive_motors = motor_group(RS_front, RS_back);
inertial IMU1 = inertial(PORT14);
encoder TY = encoder(Brain.ThreeWirePort.A);
encoder TX = encoder(Brain.ThreeWirePort.A);
potV2 sweeper_pot = potV2(Brain.ThreeWirePort.H);
//
*/



extern task motor_control;
  data bot(6.756286489, 0.0, 1.00, true);
  //data bot(6.756286489, 0.0, 1.00, false);
  PID sweeper_pid(.1,0,0);
  sweeper_class sweep;
  PID drive_pid(0, 0, 0);
  PID turn_pid(0, 0, 0);
  
int odometry_thread_wrapper() {
  bot.odom_thread();
  return 0;
}
void pre_auton(void) {
  IMU1.calibrate();
  task motor_control(motor_thread);
  task brain_screen(brain_readout_thread);
  task terminal(wireless_readout_thread);
  task odometry(odometry_thread_wrapper);
bot.reset_sensors();
bot.setPos(0,0,0);
}

void autonomous(void) {
        enable_user_control = false;
//test_auto();
prog_skills_15();
}


void usercontrol(void) {
  enable_auto_movement = false;
  drive_controllers(1, 1);
  if (bot.use_tracking_wheels) intake_controller(1);
  sweeper_controller(0);

  if (Controller.ButtonLeft.pressing()) wait(2400, vex::msec), autonomous();
  while (1) {
      enable_user_control = true;
    user_drive_control();
    wait(10, msec);
  }
}

int main() {
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true) {
    wait(100, msec);
  }
}
