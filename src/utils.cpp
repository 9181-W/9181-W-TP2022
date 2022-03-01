#include "okapi/api.hpp"
using namespace okapi;
#include "initialize.hpp"
#include "utils.hpp"
#include "inertial.hpp"
// #include "async_curve_drive_to_point.hpp"

pros::Task* intake_task = NULL;
double async_complete_4;
double async_intake_speed = 0.0;

//Intake_functions

void stop_intake()
{
  intake_task->remove();
  intake_task = NULL;
  intake_mtr.moveVelocity(0);
  async_complete_4 = true;
}

void intake(double async_intake_speed)
{
  int start_time = pros::c::millis();
  while(async_complete_4 == false)
  {
     // - start_time) < 2000
    if((intake_mtr.getActualVelocity() < 15) && ((pros::c::millis() - start_time) > 2000))
    {
      intake_mtr.moveVelocity(-600);
      pros::delay(500);
      intake_mtr.moveVelocity(async_intake_speed);
      start_time = pros::c::millis();
    }
    else
    {
      intake_mtr.moveVelocity(async_intake_speed);
    }
    pros::delay(33);
  }
}

void async_intake(void *param)
{
  while (true)
  {
    //if the drive is not complete continue running the drive
    if(!async_complete_4)
    {
      intake(async_intake_speed);
      async_complete_4 = true;
    }
    pros::delay(33);
  }
}

void async_intake(double intake_speed)
{
  async_intake_speed = intake_speed;

  //runs as an asynchronous task
  if (intake_task == NULL)
  {
    intake_task = new pros::Task(async_intake, (void*)"PROSCLAW", TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Async claw Task");
  }
  async_complete_4 = false;
}


//Arm Functions

void move_arm(double position, double speed)
{
  arm_mtr.moveAbsolute(position, speed);
}

void lower_arm(double speed, bool wait_for_complete)
{
  arm_mtr.moveAbsolute(0, speed);
  if (wait_for_complete == true)
  {
    do {
      pros::delay(33);

      // printf("Is stopped: %f\n",lifter_smtr.getPosition());
    } while((wait_for_complete == true) && (arm_mtr.getPosition() >= 5));
  }
}

//Lifter Functions

void lower_lifter()
{
  tilter.set_value(true);
  pros::delay(300);
  back_claw.set_value(false);
}

void raise_lifter()
{
  back_claw.set_value(true);
  pros::delay(125);
  tilter.set_value(false);
}

void lower_flap()
{
  pneumatic_flap.set_value(true);
}

void raise_flap()
{
  pneumatic_flap.set_value(false);
}

/*
pros::Task* lifter_task = NULL;
double async_complete_7;

void lower_lifter(double speed, bool wait_for_complete)
{
  int start_time = pros::millis();
  pros::delay(12);
  while((lifter_mtr.getActualVelocity() < 0) || ((pros::c::millis() - start_time) < 500))
  {
    lifter_mtr.moveVelocity(speed);
    pros::delay(33);
  }
}

void async_lifter_shut_off()
{
  lifter_task->remove();
  lifter_task = NULL;
}

void async_lower_lifter(void *param)
{
  while (true)
  {
    //if the drive is not complete continue running the drive
    if(!async_complete_7)
    {
      lower_lifter(-100, false);
      async_complete_7 = true;
    }
    pros::delay(10);
  }
}

void async_lower_lifter()
{
  //runs as an asynchronous task
  if (lifter_task == NULL)
  {
    lifter_task = new pros::Task(async_lower_lifter, (void*)"PROSCLAW", TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Async Lifter Task");
  }
  async_complete_7 = false;
}

void raise_lifter(double speed)
{
  pros::delay(12);
  while(limit_switch.get_value() == false)
  {
    lifter_mtr.moveVelocity(-speed);
    pros::delay(33);
  }
  lifter_mtr.moveVelocity(0);
}
*/


//Claw Functions

void open_claw()
{
  pneumatic_claw.set_value(true);
}

void close_claw()
{
  pneumatic_claw.set_value(false);
}

/*
pros::Task* claw_task = NULL;
double async_complete_3;

void claw_actuate()
{
  // claw_task->resume();
  while (claw_ultrasonic.get_value() > 58.0 || claw_ultrasonic.get_value() < 10.0)
  {
    pneumatic_claw.set_value(true);
    pros::delay(33);
    pros::lcd::print(4,"ultrasonic %5.1f", claw_ultrasonic.get_value());
  }
  pneumatic_claw.set_value(false);
  // claw_task->remove();
  // claw_task = NULL;
  kill_drive_task();
}

void async_claw_shut_off()
{
  claw_task->remove();
  claw_task = NULL;
}

void async_claw_actuate(void *param)
{
  while (true)
  {
    //if the drive is not complete continue running the drive
    if(!async_complete_3)
    {
      claw_actuate();
      async_complete_3 = true;
    }
    pros::delay(33);
  }
}

void async_claw_actuate()
{
  //runs as an asynchronous task
  if (claw_task == NULL)
  {
    claw_task = new pros::Task(async_claw_actuate, (void*)"PROSCLAW", TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Async claw Task");
  }
  async_complete_3 = false;
}
*/

void kill_all_tasks()
{
  // if(claw_task != NULL)
  // {
  //   claw_task->remove();
  //   claw_task = NULL;
  //   pneumatic_claw.set_value(true);
  // }
  if(intake_task != NULL)
  {
    intake_task->remove();
    intake_task = NULL;
    intake_mtr.moveVelocity(0);
  }
  // if(lifter_task != NULL)
  // {
  //   lifter_task->remove();
  //   lifter_task = NULL;
  //   lifter_mtr.moveVelocity(0);
  // }
  // kill_drive_task();
}

void deploy_claw(bool on)
{
  pneumatic_claw.set_value(on);
}
