#pragma once
extern bool left_drive_controller_on, right_drive_controller_on, intake_controller_on, sweeper_controller_on;
void drive_controllers(bool left_on, bool right_on);
void intake_controller(bool on);
void sweeper_controller(bool on);
void left_drive(float power, bool vel_control, int stopping);
void right_drive(float power, bool vel_control, int stopping);
void intake_at(float power, bool vel_control, int stopping);
void sweep_at(float power, int stopping);

void spin_motors();
int motor_thread();