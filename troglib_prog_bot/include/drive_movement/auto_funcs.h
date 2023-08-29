#pragma once
extern double start_time;
extern double end_time;
extern double temporary_multiplier;

void start_auto(float x, float y, float h);
void stop_auto();
void delay(float msec);
void movement_reset();
void odomTune(float kp, float ki, float kd, float maxSpeed);
void turn_to(float ang, float kp, float ki, float kd, float maxSpeed, float breakang);
void turnToExplicit(float ang, float kp, float ki, float kd, float maxSpeed, float breakang);
void tuneOffsets(float ang, float kp, float ki, float kd, float maxSpeed, float breakang);
void turn_to_xy(float x, float y, float kp, float ki, float kd, float maxSpeed, float breakang, bool backward);
void swing_on_left(float ang, float kp, float ki, float kd, float maxSpeed, float breakang);
void swing_on_right(float ang, float kp, float ki, float kd, float maxSpeed, float breakang);
void straight(float dist, float kp, float ki, float kd, float maxSpeed, float slew, float breakdist);
void straight_with_heading(float dist, float heading, float ykp, float yki, float ykd, float hkp, float hkd, float maxSpeed, float hmaxspeed, float slew, float breakdist);
void classic_move_to(float x, float y, float ymax, float hmax, float ykp, float hkp, float slew, float breakLength, bool backwards);
