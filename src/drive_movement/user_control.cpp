#include "motor_controller.h"
#include "robot_config.h"
#include "vex.h"

float axis1, axis2, axis3, axis4;
bool R1, R2, L1, L2, last_R1, last_R2, last_L1, last_L2;
bool A, B, X, Y, last_A, last_B, last_X, last_Y;
bool LEFT, RIGHT, UP, DOWN, last_LEFT, last_RIGHT, last_UP, last_DOWN;
bool toggle = false;
bool override_drive = false;
void collect_input()
{
    axis1 = Controller.Axis1.position();
    axis2 = Controller.Axis2.position();
    axis3 = Controller.Axis3.position();
    axis4 = Controller.Axis4.position();
    R1 = Controller.ButtonR1.pressing();
    R2 = Controller.ButtonR2.pressing();
    L1 = Controller.ButtonL1.pressing();
    L2 = Controller.ButtonL2.pressing();
    A = Controller.ButtonA.pressing();
    B = Controller.ButtonB.pressing();
    X = Controller.ButtonX.pressing();
    Y = Controller.ButtonY.pressing();
    LEFT = Controller.ButtonLeft.pressing();
    RIGHT = Controller.ButtonRight.pressing();
    UP = Controller.ButtonUp.pressing();
    DOWN = Controller.ButtonDown.pressing();
}
void last_input()
{
    last_R1 = R1;
    last_R2 = R2;
    last_L1 = L1;
    last_L2 = L2;
    last_A = A;
    last_B = B;
    last_X = X;
    last_Y = Y;
    last_LEFT = LEFT;
    last_RIGHT = RIGHT;
    last_UP = UP;
    last_DOWN = DOWN;
}

void arcade()
{
    float left = axis3 + axis4;
    left = left * .12;
    float right = axis3 - axis4;
    right = right * .12;
    left_drive(left, 0, 0);
    right_drive(right, 0, 0);
}

void tank()
{
    float left = axis3;
    left = (left * fabs(left) / 100) * .12;
    float right = axis2;
    right = (right * fabs(right) / 100) * .12;
    left_drive(left, 0, 0);
    right_drive(right, 0, 0);
}

void user_intake()
{
    float pow = (R1 - R2) * 12;
    intake_at(pow, 0, 1);
}

void turn_macros()
{
    if (A)
    {
        override_drive = true;
        left_drive(3, 0, 0);
        right_drive(-3, 0, 0);
    }
    else if (X)
    {
        override_drive = true;
        left_drive(-3, 0, 0);
        right_drive(3, 0, 0);
    } else
    {
        override_drive = false;
    }
}

void user_drive_control()
{
    collect_input();
    turn_macros();
    if (B && B != last_B)
        toggle = !toggle;
    if (!override_drive) {
        if (toggle)
            arcade();
        else
            tank();
    }


    user_intake();

    last_input();
}