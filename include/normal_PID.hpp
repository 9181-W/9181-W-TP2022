#include "okapi/api.hpp"
using namespace okapi;

void gyro_drive(QLength distance, double max_speed, bool drive_straight = true);
void async_gyro_drive(QLength distance, double max_speed);
bool is_drive_complete();
void wait_for_drive_complete();
