#include "okapi/api.hpp"
using namespace okapi;

// void gyro_drive(QLength distance, double max_speed, bool drive_straight = true);
void gyro_drive(QLength distance, double drive_kp, double max_speed, double min_speed, bool drive_straight, double drive_straight_kp, double epsilon);
// void async_gyro_drive(QLength distance, double max_speed);
void async_gyro_drive(QLength distance, double drive_kp, double max_speed, double min_speed, bool drive_straight, double drive_straight_kp, double epsilon);
bool is_drive_complete();
void wait_for_drive_complete();
