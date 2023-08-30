#pragma once;

class MP_move
{
    private:
float turn_disable_distance = 5;
float classic_turn_margin = 80;

bool first;
public:
    float mp_disable_length = 4;
    float initial_y_tracker_inches;
    float straight_mp_length;
    float direction_multiplier;
    float error;
    float travelled;
    bool decellerating = false;
    float acel, dist, max_speed;
    float current_target_acel;
    float output;
    double linearError2D();
    float TurnErrorFrom(float targ);
    float acelDist(float initial_v, float final_v, float a);
    float mp_1d_speed(float dist, float max_v, float acel);
    void straight();
    void classicToPoint();
};

extern MP_move mp_calc;