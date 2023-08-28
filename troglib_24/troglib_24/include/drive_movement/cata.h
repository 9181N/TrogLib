#pragma once
int cata_thread();
class cata // versatile PID class
{
  private:
  float count = 0;
  public:
  bool shoot_cata;
  float error;
    void cata_middle();
    void cata_down();
    void cata_disable();
    void cata_enable();
    void wait_until_down(float target);
    void shoot();
    void power();
};

extern cata catapult;