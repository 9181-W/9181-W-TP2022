#include "okapi/api.hpp"
using namespace okapi;

enum
{
    RED_GOAL = 0,
    BLUE_GOAL,
    YELLOW_GOAL,
    NONE
};

extern void vision_drive(int colour, double max_speed, double min_speed, double distance_to_mogo, double drive_straight_kp);
// void async_gyro_drive(QLength distance, double max_speed);
// bool is_drive_complete();
// void wait_for_drive_complete();
