#include "okapi/api.hpp"
using namespace okapi;

void drive_through_point(QLength x_target, QLength y_target, double max_speed, double min_speed, double kp, double kd, double max_turn_speed, double turn_kp, bool reverse, double rotate_epsilon, double epsilon, double division_const);
