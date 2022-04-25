#include "okapi/api.hpp"
using namespace okapi;

void async_drive_through_point(QLength x_target, QLength y_target, double max_speed, double max_turn_speed, double turn_kp, bool reverse, double epsilon, double lookahead_x, double lookahead_y, double lookahead_percentage);
void wait_for_drive_complete_6();
void drive_is_complete_6();
// void kill_drive_task();s
