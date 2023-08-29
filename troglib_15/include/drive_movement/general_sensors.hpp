#pragma once

#include "mkhlib/datatypes.hpp"
#include "vex.h"

namespace mkhlib {
    
    class UnboundAngularTracker {
        
        bool is_rotation = true;

        void set_value(double value, vex::rotationUnits units);
        double get_value(vex::rotationUnits units);
    };

    // clamped between 0 & 360
    class BoundAngularTracker {

        bool is_rotation = true;

        void set_value(Angle value);
        Angle get_value();
    };
}