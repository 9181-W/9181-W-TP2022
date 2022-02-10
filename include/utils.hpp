#include "okapi/api.hpp"
using namespace okapi;


// extern void intake(double intake_speed);
extern void async_intake(double async_intake_speed);
extern void stop_intake();
extern void move_arm(double position, double speed);
// extern void lower_lifter(double speed, bool wait_for_complete);
// extern void raise_lifter(double speed);
extern void lower_lifter();
extern void raise_lifter();
// extern void deploy_claw(bool on);
// extern void claw_actuate();
// extern void async_claw_actuate();
extern void open_claw();
extern void close_claw();
extern void kill_all_tasks();
// extern void async_claw_shut_off();
// extern void async_lower_lifter();
extern void lower_arm(double speed, bool wait_for_complete);
