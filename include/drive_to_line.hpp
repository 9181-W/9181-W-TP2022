#include "okapi/api.hpp"
using namespace okapi;

void curve_drive_to_line(QLength x_target, QLength y_target, double max_speed, double min_speed, double kp, double kd, double max_turn_speed, double turn_kp, bool reverse, double epsilon);
