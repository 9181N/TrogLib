#pragma once

#include <vector>
#include "v5.h"
#include "v5_vcs.h"

#include "mkhlib/datatypes.hpp"
#include "mkhlib/general_sensors.hpp"
#include "mkhlib/pid.h"

namespace mkhlib {
    class MotionController {
        public:

        // CONSTRUCTORS

        MotionController(std::vector<int> left_ports, std::vector<int> right_ports, 
                            mkhlib::UnboundAngularTracker *lateral_tracker_ref, mkhlib::UnboundAngularTracker *horizontal_tracker_ref,
                            int inertial_port, double lateral_offset, double horizontal_offset, double lateral_diameter, 
                            double horizontal_diameter) 
        : inertial(abs(inertial_port)), horizontal_tracker(horizontal_tracker_ref), lateral_tracker(lateral_tracker_ref),
        lateral_PID(0,0,0), angular_PID(0,0,0), lateral_tracker_offset(lateral_offset), lateral_wheel_diameter(lateral_diameter),
        horizontal_tracker_offset(horizontal_offset), horizontal_wheel_diameter(horizontal_diameter)
        {
            odometry_task = vex::task(odometry_process);
            auto_task = vex::task(auto_process);
        }

        // END CONSTRUCTORS


        // ELECTRONICS

        vex::motor_group left_motors;
        vex::motor_group right_motors;

        vex::inertial inertial;

        UnboundAngularTracker* lateral_tracker;
        UnboundAngularTracker* horizontal_tracker;

        // END ELECTRONICS

        // PIDS

        PID lateral_PID;
        PID angular_PID;

        // END PIDS

        // ODOMETRY VALUES

        Vector2 position;
        Angle orientation;

        // END ODOMETRY VALUES

        private:

        // TASK FUNCTIONS

        static int odometry_process();
        static int auto_process();

        // END TASK FUNCTIONS

        // TASKS

        vex::task odometry_task;
        vex::task auto_task;

        // END TASKS

        // ODOMETRY CALCULATION CONSTANTS

        double lateral_tracker_offset;

        double horizontal_tracker_offset;

        double lateral_wheel_diameter;

        double horizontal_wheel_diameter;

        // END ODOMETRY CALCULATION CONSTANTS

    };
}
