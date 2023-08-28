#pragma once
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
    float maxintegral, integralbound, integralmargin;
    float cur, targ, error = 0, lasterror = 0, deltaerror, sumerror;
    float output, maxOutput, prevOutput;
    float slewAmount;
    void reset();
    void setIntegral(float cap, float bound);
    void setConstants(float KP, float KI, float KD);
    float signum(float input);
    float calculate(float inputError);
};


extern PID turn_pid;
extern PID drive_pid;
extern PID sweeper_pid;
extern PID cata_pid;