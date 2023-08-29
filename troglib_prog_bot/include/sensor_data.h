#pragma once
class data
{
public:
    double x, y, h_deg, h_rad, last_h_rad;
    double hy, hx;
    double x_target, y_target, h_target;
    double parallel_deg, parallel_inch, parallel_deg_r, parallel_inch_r, perpindicular_deg, perpindicular_inch;
    double last_parallel_deg, last_parallel_inch, last_perpindicular_deg, last_perpindicular_inch, delta_parallel_inch, delta_perpindicular_inch;
    double linear_speed, last_linear_speed;
    double SS, SR;
    bool use_tracking_wheels;
    double imu_multiplier;
    const double data_cycle_time = 10;

    data(double ss, double sr, double imu_mult, bool tracking_wheels)
    {
        SS = ss, SR = sr, imu_multiplier = imu_mult, use_tracking_wheels = tracking_wheels;
    }
    void collect_data();
    void reset_sensors();
    void setPos(float newx, float newy, float newh);
    void updateOdom();
    int odom_thread();

    double point_distance(float x1, float y1, float x2, float y2);
    double point_angle(float x1, float y1, float x2, float y2);
    double wrapangle(double ang);
    double relativeangle(float x1, float y1, float x2, float y2);
    float vector_x_length_at_theta(float theta, float vector_length);
    float vector_y_length_at_theta(float theta, float vector_length);

};

extern data bot;