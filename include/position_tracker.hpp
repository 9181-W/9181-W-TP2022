#include "okapi/api.hpp"
#include "inertial.hpp"

using namespace okapi;

// extern std::shared_ptr<ChassisController> chassis;

void tracker_initialize();
double get_x_position();
double get_y_position();
QAngle get_heading();
void reset_pos_generic(double new_x_pos, double new_y_pos);
