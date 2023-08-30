#pragma once

#include "mkhlib/datatypes.hpp"
#include "vex.h"
#include "mkhlib/utils.hpp"

namespace mkhlib {
    
    class UnboundAngularTracker {

        public:

        virtual void set_value(double value, vex::rotationUnits units) = 0;
        virtual double get_value(vex::rotationUnits units) = 0;
        
    };

    // clamped between 0 & 360
    class BoundAngularTracker {

        public:

        virtual void set_value(Angle value) = 0;
        virtual Angle get_value() = 0;
    };

    class UnboundRotationSensor : public UnboundAngularTracker {

        public:

        UnboundRotationSensor(int port)
        : rotation(abs(port), utils::negative(port)) 
        {

        }

        void set_value(double value, vex::rotationUnits units) {
            rotation.setPosition(value, units);
        }

        double get_value(vex::rotationUnits units) {
            rotation.position(units);
        }
        
        private:

        vex::rotation rotation;

    };

    class UnboundOpticalEncoder : public UnboundAngularTracker {
        public:

        UnboundOpticalEncoder(vex::triport::port &port, bool is_reversed)
        : encoder(port)
        {
            if(is_reversed) {
                reversed = -1;
            }
            else {
                reversed = 1;
            }
        }

        void set_value(double value, vex::rotationUnits units) {
            encoder.setPosition(value * reversed, units);
        }

        double get_value(vex::rotationUnits units) {
            encoder.position(units) * reversed;
        }
        
        private:

        vex::encoder encoder;
        int reversed;
    };

    class BoundRotationSensor : public BoundAngularTracker {

        public:

        BoundRotationSensor(int port)
        : rotation(abs(port), utils::negative(port)) 
        {

        }

        void set_value(Angle value) {
            rotation.setPosition(value.getDeg(), vex::rotationUnits::deg);
        }

        Angle get_value() {
            Angle::fromDeg(rotation.angle(vex::rotationUnits::deg));
        }
        
        private:

        vex::rotation rotation;

    };

    class BoundOpticalEncoder : public BoundAngularTracker {
        public:

        BoundOpticalEncoder(vex::triport::port &port, bool is_reversed)
        : encoder(port)
        {
            if(is_reversed) {
                reversed = -1;
            }
            else {
                reversed = 1;
            }
        }

        void set_value(Angle value) {
            encoder.setPosition(value.getDeg(), vex::rotationUnits::deg);
        }

        Angle get_value() {
            Angle::fromDeg(encoder.rotation(vex::rotationUnits::deg));
        }
        
        private:

        vex::encoder encoder;
        int reversed;
    };
}