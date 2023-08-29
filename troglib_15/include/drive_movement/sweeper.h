#pragma once
class sweeper_class // versatile PID class
{
  private:
  float count = 0;
  public:
     float sweeper_pid_targ;
    float sweeper_position;
    bool sweep;
    void sweep_cycle();
};

extern sweeper_class sweep;