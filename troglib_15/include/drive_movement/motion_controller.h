#include <vector>
#include "v5.h"
#include "v5_vcs.h"

#include "mkhlib/datatypes.hpp"

class MotionController {
    public:

    // CONSTRUCTORS



    // END CONSTRUCTORS


    // ELECTRONICS

    vex::motor_group left_motors;
    vex::motor_group right_motors;

    vex::inertial inertial;

    // END ELECTRONICS

    // PIDS

    // END PIDS

    // ODOMETRY VALUES

    Vector2 position;
    Angle orientation;

    // END ODOMETRY VALUES


    private:

};