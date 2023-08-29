#pragma once

int sweeper_thread();


class sweeper_class // versatile PID class

{
  private:
  float count = 0;
  public:
     float sweeper_pid_targ;
    float sweeper_position;
    float error;
    bool sweep;
    void move_sweeper(bool out);
    void sweep_cycle();
    void wait_until_error(float margin);

};

extern sweeper_class sweep;