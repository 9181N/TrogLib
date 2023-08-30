#include "motor_controller.h"
#include "drive_movement/pid_movement.h"
#include "drive_movement/motion_profile.h"
#include "vex.h"
#include <iostream>
bool enable_auto_movement = false;
int movement_type_index = 0;

int auto_left_drive_stopping, auto_right_drive_stopping;
bool auto_left_drive_vel, auto_right_drive_vel;
double auto_left_drive_power, auto_right_drive_power;
bool backwards_move;




void auto_movement()
{
    if (movement_type_index > 0)
    {

switch(movement_type_index){
case 1: pid_calc.straight(); break;
case 2: pid_calc.straight_heading(); break;
case 3: pid_calc.turn(); break;
case 4: pid_calc.turn_to_xy(); break;
case 5: pid_calc.swing_turn_on_left(); break;
case 6: pid_calc.swing_turn_on_right(); break;
case 7: pid_calc.swing_turn_to_xy_on_left(); break;
case 8: pid_calc.swing_turn_to_xy_on_right(); break;
case 9: pid_calc.straight_heading_to_point(); break;
case 10: pid_calc.classic_to_point(); break;
case 11: mp_calc.straight(); break;
/*case 12: PPSBezierCalc(); break;
case 13: simplePPSBezierCalc(); break;
case 14: backwardsPPSBezierCalc(); break;
case 15: simplePPSBezierCalcBackward(); break;
case 16: straightlinecalc(); break;
case 17: straightheadingcalc(); break;
case 18: straightheadingcalcToPoint(); break;
case 19: motion_profile_calc(); break;*/
default: break;
}

        if (enable_auto_movement) 
         {
            left_drive(auto_left_drive_power, auto_left_drive_vel, auto_left_drive_stopping);
            right_drive(auto_right_drive_power, auto_right_drive_vel, auto_right_drive_stopping);
         }

    }
}