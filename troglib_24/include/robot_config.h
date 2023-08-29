#pragma once
#include "vex.h"

extern vex::brain Brain;
extern vex::controller Controller;
extern vex::motor_group left_drive_motors;
extern vex::motor_group right_drive_motors;
extern vex::motor_group cata_motors;

extern vex::motor intake;
extern vex::motor sweeper;
extern vex::inertial IMU1;
extern vex::optical color_sensor;
extern vex::encoder TY;
extern vex::encoder TX; 
extern vex::rotation cata_rotation;

extern vex::potV2 sweeper_pot; 