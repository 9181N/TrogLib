#include "robot_config.h"
#include "drive_movement/auto_movement_loop.h"
#include "vex.h"
#include <iostream>
using namespace vex;
float left_drive_power, right_drive_power, intake_power, sweeper_power, cata_power;
bool left_drive_controller_on, right_drive_controller_on, intake_controller_on, sweeper_controller_on, cata_controller_on;
bool left_drive_vel_control, right_drive_vel_control, intake_vel_control;
int left_drive_stopping = 0, right_drive_stopping = 0, intake_stopping = 0, sweeper_stopping = 0, cata_stopping = 0;

void drive_controllers(bool left_on, bool right_on) {
left_drive_controller_on = left_on, right_drive_controller_on = right_on;
}
void intake_controller(bool on) {
intake_controller_on = on;
}

void sweeper_controller(bool on) {
sweeper_controller_on = on;
}

void cata_controller(bool on) {
cata_controller_on = on;
if (!on) cata_motors.stop(vex::coast);
}


void left_drive(float power, bool vel_control, int stopping) {
    left_drive_power = power, left_drive_vel_control = vel_control, left_drive_stopping = stopping;
}
void right_drive(float power, bool vel_control, int stopping) {
    right_drive_power = power, right_drive_vel_control = vel_control, right_drive_stopping = stopping;
}
void intake_at(float power, bool vel_control, int stopping) {
    intake_power = power, intake_vel_control = vel_control, intake_stopping = stopping;
}

void sweep_at(float power, int stopping) {
    sweeper_power = power, sweeper_stopping = stopping;
}

void cata_at(float power, int stopping) {
    cata_power = power, cata_stopping = stopping;
}


void left_controller() {
if (left_drive_power != 0) {
    if (left_drive_vel_control) left_drive_motors.spin(fwd, left_drive_power, pct);  
    if (!left_drive_vel_control) left_drive_motors.spin(fwd, left_drive_power, volt);  
    } 
    else {
        if (left_drive_stopping == 2) left_drive_motors.stop(hold);
        else if (left_drive_stopping == 1) left_drive_motors.stop(brake);
        else left_drive_motors.stop(coast);
    }
}

void right_controller() {
if (right_drive_power != 0) {
    if (right_drive_vel_control) right_drive_motors.spin(fwd, right_drive_power, pct);  
    if (!right_drive_vel_control) right_drive_motors.spin(fwd, right_drive_power, volt);  
    } 
    else {
        if (right_drive_stopping == 2) right_drive_motors.stop(hold);
        else if (right_drive_stopping == 1) right_drive_motors.stop(brake);
        else right_drive_motors.stop(coast);
    }
}

void intake_controller_func() {
if (intake_power != 0) {
    if (intake_vel_control) intake.spin(fwd, intake_power, pct);  
    if (!intake_vel_control) intake.spin(fwd, intake_power, volt);  
    } 
    else {
        if (intake_stopping == 2) intake.stop(hold);
        else if (intake_stopping == 1) intake.stop(brake);
        else intake.stop(coast);
    }
}

void sweeper_controller_func() {
if (sweeper_power != 0) {
   sweeper.spin(fwd, sweeper_power, volt);  
    } 
    else {
        if (sweeper_stopping == 2) sweeper.stop(hold);
        else if (sweeper_stopping == 1) sweeper.stop(brake);
        else sweeper.stop(coast);
    }
}

void cata_controller_func() {
if (cata_power != 0) {
   cata_motors.spin(fwd, cata_power, volt);  
    } 
    else {
        if (cata_stopping == 2) cata_motors.stop(hold);
        else if (cata_stopping == 1) cata_motors.stop(brake);
        else cata_motors.stop(coast);
    }
}



void spin_motors() {
if (left_drive_controller_on) left_controller();
if (right_drive_controller_on) right_controller();
if (intake_controller_on) intake_controller_func();
if (sweeper_controller_on) sweeper_controller_func();
if (cata_controller_on) cata_controller_func();
else cata_motors.setStopping(vex::coast);
}

int motor_thread() {
    while (1) {
        auto_movement();
        spin_motors();
        wait(10, vex::msec);
    }
    return 0;
}