#pragma once

class MP_move
{
public:
    bool first = false;
    bool disable_acel = false;
    float turn_disable_distance = 5;
    float classic_turn_margin = 70;
    float mp_disable_length = 4;
    float initial_y_tracker_inches;
    float straight_mp_length;
    float direction_multiplier;
    float error;
    float travelled;
    bool decellerating = false;
    float acel, dist, max_speed, initial_dist;
    float current_target_acel;
    float output;
    float pow = 0, left_pow = 0, right_pow = 0;
    double linearError2D();
    float TurnErrorFrom(float targ);
    float acelDist(float initial_v, float final_v, float a);
    float mp_1d_speed(float dist, float max_v, float acel, bool adaptive);
    void straight();
    void classicToPoint();
    void pathFollow();
};

extern MP_move mp_calc;