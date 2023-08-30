//kv, ka, kd, kp
//FF = (Kv * target vel) + (ka * target accel)
//FB = Kp * (target vel - measured vel)
//(FF + FB)
//Pick a Kv approximately equal to 1/top speed With the other constants set to 0
//Adjust Kv until the target and measured velocities match up when the robot is traveling a constant speed.
//Start Ka around 0.002. Adjust until the target and measured velocities match decently (don’t worry about perfection) when accelerating, cruising, and decelerating.
//kp We used 0.01. Feel free to make it larger, as the robot will more accurately track the target velocities. But be warned: if you increase it too much it will become jittery.
class velo_controller {
    public:
float Kv, Ka, Kd, Kp;
velo_controller(float kv, float ka, float kd, float kp) {
Kv = kv, Ka = ka, Kd = kd, Kp = kp;
}

float i = 0;
float signum(float input);
float motor_power(float target_speed, float target_acceleration, float current_speed);


};

extern velo_controller left_side_drive;
extern velo_controller right_side_drive;