#include "vex.h"
#include "motor_controller.h"
#include "drive_movement/user_control.h"
#include "data_output/brain_screen.h"
#include "data_output/wireless_terminal.h"
#include "sensor_data.h"
#include "drive_movement/pid.h"
#include "drive_movement/auto_movement_loop.h"
#include "autos.h"
using namespace vex;
brain Brain;
controller Controller;
competition Competition;

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


extern task motor_control;
  data bot(6.756286489, 0.0, 1.00);
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
test_auto();
}


void usercontrol(void) {
  enable_auto_movement = false;
  drive_controllers(1, 1);
  intake_controller(1);
  while (1) {
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
