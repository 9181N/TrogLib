/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ingim                                                     */
/*    Created:      8/28/2023, 9:46:01 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "mkhlib/lib.hpp"
#include "mkhlib/motion_controller.hpp"

#include <vector>

vex::brain Brain;

mkhlib::MotionController movehammadAli(
    // ELECTRONICS

    
    // Left motor ports
    {4, -2, 6}, 
    
    // Right motor ports
    {1, 2, 3}, 
    
    // Lateral encoder reference 
    &mkhlib::UnboundRotationSensor(1), 
    
    // Horizontal encoder reference
    &mkhlib::UnboundOpticalEncoder(Brain.ThreeWirePort.A, false), 
    
    // Inertial sensor port
    1,
    
    // CHASSIS CONSTANTS
    
    
    // lateral tracker offset
    0.0,
    
    // horizontal tracker offset
    6.756286489,
    
    // lateral wheel diameter
    2.75,

    // horizontal wheel diameter
    2.75
    
    );
int main() {

}