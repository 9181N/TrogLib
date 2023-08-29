#include "drive_movement/auto_funcs.h"
#include "sensor_data.h"
#include "drive_movement/cata.h"
#include "drive_movement/sweeper.h"
#include "motor_controller.h"
#include "vex.h"
#include <iostream>

float ykp = .8, yki = 0.0, ykd = 8;
float hkp = .28, hki = .01, hkd = 3;

void test_auto()
{
    start_auto(0, 0, 0);
    cata_controller(0);
    straight(24, ykp, yki, ykd, 12, .125, 1);
    delay(300);
    straight_with_heading(-24, 0, ykp, yki, ykd, hkp, hkd, 12, 12, .125, 1);
    classic_move_to(-20, 30, 9, 12, ykp, hkp * .6, .125, 2, false);
    delay(300);
    // turn_to_xy(0,0, hkp, hki, hkd, 6, 1, false);
    // delay(300);
    classic_move_to(0, 0, 9, 12, ykp, hkp, .125, 2, true);
    delay(500);
    turn_to(0, hkp, hki, hkd, 9, 1);

    // straight(-24, ykp, yki, ykd, 12, .15, 1);
    // straight_with_heading(-24, -90, ykp, yki, ykd, hkp, hkd, 12, 12, .125, 1);

    /*
        turn_to(90, hkp, hki, hkd, 12, 1);
        turn_to(0, hkp, hki, hkd, 12, 1);
        turn_to_xy(-45, 45, hkp, hki, hkd, 12, 1, false);
        turn_to_xy(-45, 45, hkp, hki, hkd, 12, 1, true);
        turn_to(0, hkp, hki, hkd, 12, 1);
    */
    stop_auto();
}

void sweep_once()
{
    sweep.move_sweeper(0);
    sweep.wait_until_error(20);
    sweep.move_sweeper(1);
    sweep.wait_until_error(20);
}

void matchload_cycle() {
    catapult.wait_until_down(15);
    sweep.move_sweeper(0);
    sweep.wait_until_error(20);
    sweep.move_sweeper(1);
    sweep.wait_until_error(20);
    catapult.shoot();
}

void matchload_x_times(float x) {
int i;

for (i = 0; i < x; i++) {
    matchload_cycle();
  printf("%d\n", i);
}

}
void prog_skills_24()
{
    catapult.cata_middle();
    start_auto(0, 0, 0);
    sweep.move_sweeper(1);
    ykp = .7, yki = 0.0, ykd = 6;
    hkp = .28, hki = .01, hkd = 3;


    turn_to(-45, hkp, hki, hkd, 10, 3);
    straight_with_heading(-23, -45, ykp, yki, ykd, hkp, hkd, 10, 12, .125, 3);
    turn_to(28, hkp, hki, hkd, 12, 3);
    straight_with_heading(40, 28, ykp, yki, ykd, hkp, hkd, 5, 12, .125, 0);
    delay(1300);
    catapult.cata_down();
    delay(1000);
    turn_to(28, hkp, hki, hkd, 12, 0);
    delay(500);
    straight_with_heading(-3, 26, ykp, yki, ykd, hkp, hkd, 5, 12, .125, 0);
    delay(500);
    swing_on_left(26, hkp, hki, hkd, 12, 0);
    // sweeper_out();
    // catapult.cata_down();
    matchload_x_times(23);

    stop_auto();



    
/*
    swing_on_right(-115, hkp * 1.2, hki, hkd, 12, 8);
    classic_move_to(85, -34, 8, 12, ykp, hkp * 1.5, .125, 4, true);
    turn_to_xy(72, -51, hkp, hki, hkd, 12, 3, false);
    delay(500);
    classic_move_to(72, -51, 4, 12, ykp, hkp * 1.5, .125, 4, false);
    delay(500);
    turn_to(-151, hkp, hki, hkd, 5, 3);
    straight_with_heading(6, -151, ykp, yki, ykd, hkp, hkd, 2.5, 12, .125, 0);
    delay(1000);
    turn_to(-151, hkp, hki, hkd, 5, 3);
    delay(2000);*/

    // classic_move_to(0, -50, 9, 12, ykp, hkp * .6, .125, 2, true);
}