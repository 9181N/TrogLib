#include <vector>
#include "v5.h"
#include "v5_vcs.h"

#include "mkhlib/datatypes.hpp"
#include "mkhlib/general_sensors.hpp"

namespace mkhlib {
    class MotionController {
        public:

        // CONSTRUCTORS

        MotionController(std::vector<int> left_ports, std::vector<int> right_ports, 
                            mkhlib::UnboundAngularTracker *lateral_tracker_ref, mkhlib::UnboundAngularTracker *horizontal_tracker_ref,
                            int inertial_port, double lateral_offset, double horizontal_offset, double lateral_diameter, 
                            double horizontal_diameter) 
        : inertial(abs(inertial_port))
        {
            lateral_tracker = lateral_tracker_ref;
        }

        // END CONSTRUCTORS


        // ELECTRONICS

        vex::motor_group left_motors;
        vex::motor_group right_motors;

        vex::inertial inertial;

        mkhlib::UnboundAngularTracker* lateral_tracker;

        // END ELECTRONICS

        // PIDS

        

        // END PIDS

        // ODOMETRY VALUES

        Vector2 position;
        Angle orientation;

        // END ODOMETRY VALUES

        private:

        // ODOMETRY CALCULATION CONSTANTS

        double lateral_tracker_offset;

        double horizontal_tracker_offset;

        double lateral_wheel_diameter;

        double horizontal_wheel_diameter;

        // END ODOMETRY CALCULATION CONSTANTS

    };
}
