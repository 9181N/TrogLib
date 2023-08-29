#include "drive_movement/pid_movement.h"
#include "sensor_data.h"
#include "math.h"
#include "drive_movement/pid.h"
#include "motor_controller.h"
#include "drive_movement/auto_movement_loop.h"
#include "vex.h"
#include "drive_movement/cata.h"
#include "robot_config.h"
#include <iostream>



void cata::wait_until_down(float target) {
    while(fabs(error) > target) {
       // printf("\n E:%.2f", error);
        wait(10, vex::msec);
    }
}

void cata::shoot()
{
    cata_controller_on = false;
    wait(10, vex::msec);
    cata_motors.setStopping(vex::coast);
    //cata_motors.setVelocity(100, vex::pct);
    //cata_motors.spinFor(600, vex::deg, true);
    cata_motors.spin(vex::fwd, 12, vex::volt);
    wait(350, vex::msec);
    cata_controller_on = true;
    shoot_cata = false;
}

void cata::cata_middle() {
    bot.cata_targ = 300;
}

void cata::cata_down() {
bot.cata_targ = 270;
}

void cata::cata_disable() {
cata_controller_on = false;
cata_motors.stop(vex::coast);
wait(20, vex::msec);
cata_motors.stop(vex::coast);
}

void cata::cata_enable() {
cata_controller_on = true;
}

void cata::power()
{
    cata_pid.maxOutput = 12;
    error = bot.cata_targ - bot.cata_position;
    float pow = cata_pid.calculate(error) * -1;
    if (bot.cata_position > bot.cata_targ + 40 or bot.cata_position < bot.cata_targ - 40) {
        pow = 12;
    }
    if (fabs(pow) < .5) pow = 0;
    cata_at(pow, 2);
}

int cata_thread()
{
    while (1)
    {
        
        if (catapult.shoot_cata) {
        catapult.shoot();
        }
        catapult.power();
        wait(10, vex::msec);
    }
    return 0;
}