#pragma once;

class pid_move
{
public:
    float initial_y_tracker_inches;
    float straight_pid_length;
    float straight_pid_error;
    bool auto_wrap_turn_target = false;
    double linear_error2d();
    float turn_error_from(float targ);
    void straight();
    void straight_heading();
    void straight_heading_to_point();
    void turn();
    void turn_to_xy();
    void swing_turn_on_left();
    void swing_turn_on_right();
    void swing_turn_to_xy_on_left();
    void swing_turn_to_xy_on_right();
    void classic_to_point();
};

extern pid_move pid_calc;