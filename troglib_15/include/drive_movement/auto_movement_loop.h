#pragma once
extern bool enable_auto_movement;
extern int movement_type_index;
extern int auto_left_drive_stopping, auto_right_drive_stopping;
extern bool auto_left_drive_vel, auto_right_drive_vel;
extern double auto_left_drive_power, auto_right_drive_power;
extern bool backwards_move;
void auto_movement();