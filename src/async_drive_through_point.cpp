#include "okapi/api.hpp"
#include "inertial.hpp"
#include "utils.hpp"
#include "position_tracker.hpp"
#include "drive_through_point.hpp"

using namespace okapi;

// void drive_through_point(QLength x_target, QLength y_target, double max_speed, double min_speed, double kp, double kd, double max_turn_speed, double turn_kp, bool reverse, double rotate_epsilon, double epsilon, double division_const)


//creates variables that will have assigned values for the asynchronous function
// std::shared_ptr<ChassisController> async_chassis_2;
QLength async_x_target_2 = 0.0_in;
QLength async_y_target_2 = 0.0_in;
double async_max_speed_2 = 0.0;
double async_max_turn_speed_2 = 0.0;
double async_turn_kp_2 = 0.0;
bool async_reverse_2 = false;
double async_epsilon_2 = 0.0;
double async_lookahead_x = 0.0;
double async_lookahead_y = 0.0;
double async_lookahead_percentage = 0.0;
bool async_complete_6 = true;
pros::Task* drive_task_6 = NULL;

//task that will run the drive to point function with new values
void async_drive_through_point(void* param)
{
  while (true)
  {
    //if the drive is not complete continue running the drive
    if(!async_complete_6)
    {
      drive_through_point(async_x_target_2, async_y_target_2, async_max_speed_2, async_max_turn_speed_2, async_turn_kp_2, async_reverse_2, async_epsilon_2, async_lookahead_x, async_lookahead_y, async_lookahead_percentage);

      //sets the drive to a completed state
      async_complete_6 = true;
    }
    pros::delay(10);
  }
}

//manual function to set the drive to a completed state
void drive_is_complete_6()
{
  async_complete_6 = false;
}

//waits until the ansychronous drive has completed
void wait_for_drive_complete_6()
{
  while(!async_complete_6)
  {
    pros::delay(10);
  }
}

// void kill_drive_task()
// {
//   // pros::c::task_suspend(drive_task_2);
//   if(drive_task_2 != NULL)
//   {
//     drive_task_2->remove();
//     delete drive_task_2;
//     drive_task_2 = NULL;
//     chassis->getModel()->stop();
//     async_complete_2 = true;
//   }
// }

//uses a new function to assign values to the asynchronous variables
void async_drive_through_point(QLength x_target_2, QLength y_target_2, double max_speed_2, double max_turn_speed_2, double turn_kp_2, bool reverse_2, double epsilon_2, double lookahead_x_2, double lookahead_y_2, double async_lookahead_percentage_2)
{
  //assigns values to the asynchronous variables
  async_x_target_2 = x_target_2;
  async_y_target_2 = y_target_2;
  async_max_speed_2 = max_speed_2;
  async_max_turn_speed_2 = max_turn_speed_2;
  async_turn_kp_2 = turn_kp_2;
  async_reverse_2 = reverse_2;
  async_epsilon_2 = epsilon_2;
  async_lookahead_x = lookahead_x_2;
  async_lookahead_y = lookahead_y_2;
  async_lookahead_percentage = async_lookahead_percentage_2;

  //runs the drive to point as an asynchronous task
  if (drive_task_6 == NULL)
  {
    drive_task_6 = new pros::Task(async_drive_through_point, (void*)"PROSDRIVE", TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Async Drive Through Point Task");
  }

  async_complete_6 = false;
}
