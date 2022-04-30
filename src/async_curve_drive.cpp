#include "okapi/api.hpp"
#include "inertial.hpp"
#include "utils.hpp"
#include "position_tracker.hpp"
#include "vision.hpp"
#include "curve_drive.hpp"
#include "drive_train.hpp"
#include "initialize.hpp"

using namespace okapi;

//creates variables that will have assigned values for the asynchronous function
// std::shared_ptr<ChassisController> async_chassis_2;
QLength async_x_target = 0.0_in;
QLength async_y_target = 0.0_in;
double async_max_speed = 0.0;
double async_min_speed = 0.0;
double async_kp = 0.0;
double async_kd = 0.0;
double async_max_turn_speed = 0.0;
double async_turn_kp = 0.0;
double async_rotate_kp = 0.0;
double async_max_rotate_speed = 0.0;
double async_min_rotate_speed = 0.0;
bool async_turn_first = false;
bool async_stop_drive = false;
bool async_reverse = false;
double async_rotate_epsilon = 0.0;
double async_epsilon = 0.0;
double async_division_const = 0.0;
int async_colour = 0.0;
double async_lookahead_distance = 0.0;
double async_true_epsilon = 0.0;
double async_straight_dist = 0.0;
bool async_complete_2 = true;
pros::Task* drive_task_2 = NULL;

//task that will run the drive to point function with new values
void async_curve_drive_to_point(void* param)
{
  while (true)
  {
    //if the drive is not complete continue running the drive
    if(!async_complete_2)
    {
      curve_drive_to_point(async_x_target, async_y_target, async_max_speed, async_min_speed, async_kp, async_kd, async_max_turn_speed, async_turn_kp, async_rotate_kp, async_max_rotate_speed, async_min_rotate_speed, async_turn_first, async_stop_drive, async_reverse, async_colour, async_rotate_epsilon, async_epsilon, async_division_const, async_lookahead_distance, async_true_epsilon, async_straight_dist);
      //sets the drive to a completed state
      async_complete_2 = true;
    }
    pros::delay(10);
  }
}

//manual function to set the drive to a completed state
void drive_is_complete_2()
{
  async_complete_2 = false;
}

//waits until the ansychronous drive has completed
void wait_for_drive_complete_2()
{
  while(!async_complete_2)
  {
    pros::delay(10);
  }
}

void kill_drive_task()
{
  // pros::c::task_suspend(drive_task_2);
  if(drive_task_2 != NULL)
  {
    drive_task_2->remove();
    delete drive_task_2;
    drive_task_2 = NULL;
    drive_train.ArcadeDrive(0.0, 0.0);
    async_complete_2 = true;
  }
}

//uses a new function to assign values to the asynchronous variables
void async_curve_drive_to_point(QLength x_target, QLength y_target, double max_speed, double min_speed, double kp, double kd, double max_turn_speed, double turn_kp, double rotate_kp, double max_rotate_speed, double min_rotate_speed, bool turn_first, bool stop_drive, bool reverse, int colour, double rotate_epsilon, double epsilon, double division_const, double lookahead_distance, double true_epsilon, double straight_dist)
{
  //assigns values to the asynchronous variables
  async_x_target = x_target;
  async_y_target = y_target;
  async_max_speed = max_speed;
  async_min_speed = min_speed;
  async_kp = kp;
  async_kd = kd;
  async_max_turn_speed = max_turn_speed;
  async_turn_kp = turn_kp;
  async_rotate_kp = rotate_kp;
  async_max_rotate_speed = max_rotate_speed;
  async_min_rotate_speed = min_rotate_speed;
  async_turn_first = turn_first;
  async_stop_drive = stop_drive;
  async_reverse = reverse;
  async_rotate_epsilon = rotate_epsilon;
  async_epsilon = epsilon;
  async_division_const = division_const;
  async_colour = colour;
  async_lookahead_distance = lookahead_distance;
  async_true_epsilon = true_epsilon;
  async_straight_dist = straight_dist;



  // async_chassis_2 = chassis;
  // async_y_distance = y_distance;
  // async_y_max_speed = y_max_speed;
  // async_y_min_speed = y_min_speed;
  // async_x_distance = x_distance;
  // async_x_max_speed = x_max_speed;
  // async_x_min_speed = x_min_speed;
  // async_target_heading = target_heading;
  // async_drive_straight_kp = drive_straight_kp;
  // async_y_drive_kp = y_drive_kp;
  // async_x_drive_kp = x_drive_kp;
  // async_turn_min_speed = turn_min_speed;

  //runs the drive to point as an asynchronous task
  if (drive_task_2 == NULL)
  {
    drive_task_2 = new pros::Task(async_curve_drive_to_point, (void*)"PROSDRIVE", TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Async Drive Task");
  }

  async_complete_2 = false;
}
