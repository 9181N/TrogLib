#include "vex.h"
#include "robot_config.h"
#include "sensor_data.h"
#include <iostream>

double deg_to_inch(double deg, double wheel_size)
{
    return (deg * wheel_size * M_PI / 360);
}
void data::collect_data()
{
    if (IMU1.isCalibrating())
    {
        h_deg = 0, h_rad = 0, last_h_rad = 0, x = 0, y = 0;
    }
    else
    {
        last_h_rad = h_rad;
        h_deg = IMU1.rotation() * imu_multiplier;
        h_rad = h_deg * M_PI / 180;
    }
    last_parallel_deg = parallel_deg, last_parallel_inch = parallel_inch, last_perpindicular_deg = perpindicular_deg, last_perpindicular_inch = perpindicular_inch;
    last_linear_speed = linear_speed;
    if (use_tracking_wheels) {
    parallel_deg = TY.position(vex::deg);
    parallel_inch = deg_to_inch(parallel_deg, 2.75);
    perpindicular_deg = TX.position(vex::deg);
    perpindicular_inch = deg_to_inch(perpindicular_deg, 2.75);
    }
    else {
    parallel_deg = left_drive_motors.position(vex::deg);
    parallel_inch = deg_to_inch(parallel_deg, 4.125);
    parallel_deg_r = right_drive_motors.position(vex::deg);
    parallel_inch_r = deg_to_inch(parallel_deg_r, 4.125);

    perpindicular_deg = 0;
    perpindicular_inch = 0;
        }
    delta_parallel_inch = parallel_inch - last_parallel_inch;
    float left_rpm = left_drive_motors.velocity(vex::rpm) * 3/5;
    float left_deg_s = left_rpm * 360 / 60;
    float right_rpm = right_drive_motors.velocity(vex::rpm) * 3/5;
    float right_deg_s = right_rpm * 360 / 60;
     left_linear_speed = deg_to_inch(left_deg_s, 3.25);
     right_linear_speed = deg_to_inch(right_deg_s, 3.25);
    linear_speed = (left_linear_speed + right_linear_speed)/2;
    //linear_speed = hy / (data_cycle_time/1000);
    delta_perpindicular_inch = perpindicular_inch - last_perpindicular_inch;
}

void data::reset_sensors()
{
    IMU1.resetRotation();
    h_deg = 0, h_rad = 0, last_h_rad = 0;
    TX.setPosition(0, vex::deg);
    TY.setPosition(0, vex::deg);
    perpindicular_deg = 0, parallel_deg = 0; // tracking wheel inputs
    x = 0, y = 0;                            // global coordinates
}

void data::setPos(float newx, float newy, float newh)
{
    reset_sensors();
    IMU1.setRotation(newh, vex::deg);
    h_deg = newh, h_rad = newh * M_PI / 180;
    last_h_rad = h_rad;
    x = newx, y = newy; // global coordinates
}

void data::updateOdom()
{
    double A = h_rad - last_h_rad;       // change in heading since last cycle
    double R = delta_parallel_inch;      // change in local Y in inches since last cycle
    double S = delta_perpindicular_inch; // change in local X in inches since last cycle


    // hy; // The hypotenuse of the triangle formed by the middle of the robot on
    // the starting position and ending position and the middle of the circle it
    // travels around
    double i; // Half on the angle that I've traveled
    // hx; // The same as h but using the back instead of the side wheels

    if (A < -0.0001 or A > 0.0001)
    {
        double ry = R / A; // The radius of the circle the robot travel's around with the right side of the robot
        double rx = S / A; // The radius of the circle the robot travel's around with the back of the robot
        i = A / 2.0;
        double sinI = sin(i);

        hy = ((ry + SR) * sinI) * 2.0;
        hx = ((rx + SS) * sinI) * 2.0;
        // thx += hx;
        // thy += hy;
    }
    else
    {
        hy = R;
        i = 0;
        hx = S;
    }
    //printf("\n A:%.6f R:%.6f S: %.6f hy:%.6f hx: %.6f hr: %.6f", A, R, S, hy, hx, last_h_rad);
    //std::cout << "\n" << std::flush;
    double p = i + h_rad; // The global ending angle of the robot
    double cosP = cos(p);
    double sinP = sin(p);
    x += hy * sinP; // update using Y tracking wheel
    y += hy * cosP;

    x += hx * cosP; // update using the X tracking wheel
    y += hx * -sinP;
}

int data::odom_thread()
{
    while (1)
    {
        collect_data();
        updateOdom();
        wait(data_cycle_time, vex::msec);
    }
}

double data::point_distance(float x1, float y1, float x2, float y2)
{
    float delta_x = x2 - x1;
    float delta_x_SQD = delta_x * delta_x;
    float delta_y = y2 - y1;
    float delta_y_SQD = delta_y * delta_y;

    float error = sqrt(delta_x_SQD + delta_y_SQD);
    return error;
}

double data::wrapangle(double ang)
{ // bounds the input to be within 180 of 0
    while (ang < -180)
    {
        ang += 360;
    }
    while (ang > 180)
    {
        ang -= 360;
    }
    return ang;
}

double data::point_angle(float x1, float y1, float x2, float y2)
{
    double ang = atan2(x2 - x1, y2 - y1);
    return ang * 180 / M_PI;
}

double data::relativeangle(float x1, float y1, float x2, float y2)
{ // calculates the relative angle from the robots current
    double ang = data::point_angle(x1, y1, x2, y2) - h_deg;
    return wrapangle(ang);
}

float data::vector_x_length_at_theta(float theta, float vector_length)
{ // literally Hcos(theta) from physics 11
    float ang = bot.wrapangle(-1*theta + 90);
    double a = ang * M_PI / 180;
    float newx = vector_length * cos(a);
    return newx;
}

float data::vector_y_length_at_theta(float theta, float vector_length)
{ // literally Hsin(theta) from physics 11
    float ang = bot.wrapangle(-1*theta + 90);
    double a = ang * M_PI / 180;
    float newy =  vector_length * sin(a);
    return newy;
}
