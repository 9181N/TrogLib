#pragma once;

class MP_move
{
    private:
float turn_disable_distance = 5;
float classic_turn_margin = 80;
bool first;
public:
    float initial_y_tracker_inches;
    float straight_pid_length;
    float straight_pid_error;
    double linearError2D();
    float TurnErrorFrom(float targ);
    float acelDist(float initial_v, float final_v, float a);
    float mp_1d_speed(float dist, float max_v, float a);
    void classicToPoint();
};

extern MP_move mp_calc;