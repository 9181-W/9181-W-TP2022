#include "okapi/api.hpp"
using namespace okapi;

void curve_drive_to_point(QLength x_target, QLength y_target, double max_speed, double min_speed, double kp, double kd, double max_turn_speed, double turn_kp, double rotate_kp, double max_rotate_speed, double min_rotate_speed, bool turn_first, bool stop_drive, bool reverse, int colour, double rotate_epsilon, double epsilon, double division_const, double lookahead_distance, double true_epsilon);
