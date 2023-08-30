#pragma once

namespace mkhlib {
  class PID // versatile PID class
  {
    private:
    float count = 0;
    public:
    PID(float KP, float KI, float KD) {
      kp = KP;
      ki = KI;
      kd = KD;
    }
    float kp, ki, kd;
    float max_integral, integral_bound, integral_margin;
    float cur, targ, error = 0, last_error = 0, delta_error, sum_error;
    float output, max_output, prev_output;
    float slew_amount;
    void reset();
    void setIntegral(float cap, float bound);
    void setConstants(float KP, float KI, float KD);
    float signum(float input);
    float calculate(float inputError);
  };
}
