#include "main.h"
#include "okapi/api.hpp"
#include "drive_train.hpp"
#include "opcontrol.hpp"
#include "initialize.hpp"
#include "normal_PID.hpp"
#include "curve_drive.hpp"
#include "utils.hpp"
#include "vision_drive.hpp"
#include "vision.hpp"
#include "turn_PID.hpp"
#include "async_curve_drive.hpp"
#include "drive_through_point.hpp"
#include "async_drive_through_point.hpp"
#include "path_following.hpp"
#include "inertial.hpp"
#include "drive_to_line.hpp"

bool L1_pressed = false; //pneumatic claw (toggle)
bool A_pressed = false; //brake mode (toggle)
bool L2_pressed = false; //pneumatic wings (toggle)
bool Y_pressed = false;
bool X_pressed = false;
bool Down_pressed = false;

void arcade_controls()
{
  double leftY = master_controller.getAnalog(okapi::ControllerAnalog::leftY);
  double leftX = ((master_controller.getAnalog(okapi::ControllerAnalog::leftX) * -1) * 0.8);

  drive_train.ArcadeDrive(leftY, leftX);
}

//controls for the front intake
void intake_controls()
{
  if ((master_controller.getDigital(okapi::ControllerDigital::R1)) == true)
  {
    intake_mtr.moveVelocity(600);
  }

  else if ((master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
  {
    intake_mtr.moveVelocity(-600);
  }
  else
  {
    intake_mtr.moveVelocity(0);
  }
}



// void intake_controls()
// {
//   if ((master_controller.getDigital(okapi::ControllerDigital::R1)) == true)
//   {
//     lifetr_mtr.moveVelocity(600);
//   }
//
//   else if ((master_controller.getDigital(okapi::ControllerDigital::R2)) == true)
//   {
//     intake_mtr.moveVelocity(-600);
//   }
//   else
//   {
//     intake_mtr.moveVelocity(0);
//   }
// }


void arm_controls(double arm_gearset_rpm = 100)
{
  const float DEAD_ZONE = 0.1;

  if (((master_controller.getAnalog(okapi::ControllerAnalog::rightY)) > DEAD_ZONE) || ((master_controller.getAnalog(okapi::ControllerAnalog::rightY)) < -DEAD_ZONE))
  {
    double rightY = master_controller.getAnalog(okapi::ControllerAnalog::rightY);

    double arm_speed = rightY;

    arm_speed *= arm_gearset_rpm;

    arm_mtr.moveVelocity(arm_speed);
  }
  else if (master_controller.getDigital(okapi::ControllerDigital::B) == true)
  {
    arm_mtr.moveAbsolute(700, 100);
  }
  else
  {
    arm_mtr.moveVelocity(0);
  }
}

bool wasAPressed = false;
bool wasXPressed = false;
bool wasL1Pressed = false;
bool wasL2Pressed = false;
bool wasYPressed = false;
bool wasDownPressed = false;
void button_utilities(void* param)
{
  /////////////////////////////////////////////UP/////////////////////////////////////////////////////
  if ((master_controller.getDigital(okapi::ControllerDigital::A)) == true)
  {
      wasAPressed = true;
  }

  if (((master_controller.getDigital(okapi::ControllerDigital::A)) == false) && wasAPressed)
  {
    wasAPressed = false;
    A_pressed = !A_pressed;

    if (A_pressed == true)
    {
      drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
    }
    if (A_pressed == false)
    {
      drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::coast);
    }
  }

  ////////////////////////////////////////////////A/////////////////////////////////////////////////
  if ((master_controller.getDigital(okapi::ControllerDigital::L1)) == true)
  {
      wasL1Pressed = true;
  }

  if (((master_controller.getDigital(okapi::ControllerDigital::L1)) == false) && wasL1Pressed)
  {
    wasL1Pressed = false;
    L1_pressed = !L1_pressed;

    if (L1_pressed == true)
    {
      pneumatic_claw.set_value(true);
    }
    if (L1_pressed == false)
    {
      pneumatic_claw.set_value(false);
    }
  }

  ////////////////////////////////////////////////X/////////////////////////////////////////////////
  if ((master_controller.getDigital(okapi::ControllerDigital::X)) == true)
  {
      wasXPressed = true;
      pros::lcd::print(2, "XPressed = %7.3f", wasXPressed);
  }

  if (((master_controller.getDigital(okapi::ControllerDigital::X)) == false) && wasXPressed)
  {
    wasXPressed = false;
    X_pressed = !X_pressed;

    if (X_pressed == true)
    {
      // pros::lcd::print(2, "wasXPressed = %7.3f", wasXPressed);
      pneumatic_flap.set_value(true);
    }
    if (X_pressed == false)
    {
      pneumatic_flap.set_value(false);
    }
  }

  ///////////////////////////////////////////X////////////////////////////////////////////////
  if ((master_controller.getDigital(okapi::ControllerDigital::L2)) == true)
  {
      wasL2Pressed = true;
  }

  if (((master_controller.getDigital(okapi::ControllerDigital::L2)) == false) && wasL2Pressed)
  {
    wasL2Pressed = false;
    L2_pressed = !L2_pressed;

    if(L2_pressed == true)
    {
      tilter.set_value(true);
      pros::delay(300);
      back_claw.set_value(false);
    }
    if(L2_pressed == false)
    {
      back_claw.set_value(true);
      pros::delay(100);
      tilter.set_value(false);
    }
  }

  ///////////////////////////////////////////DOWN////////////////////////////////////////////////
  if ((master_controller.getDigital(okapi::ControllerDigital::down)) == true)
  {
      wasDownPressed = true;
  }

  if (((master_controller.getDigital(okapi::ControllerDigital::down)) == false) && wasDownPressed)
  {
    wasDownPressed = false;
    Down_pressed = !Down_pressed;

    if(Down_pressed == true)
    {
      stick.set_value(true);
    }
    if(Down_pressed == false)
    {
      stick.set_value(false);
    }
  }

  ///////////////////////////////////////////Y////////////////////////////////////////////////
  if ((master_controller.getDigital(okapi::ControllerDigital::Y)) == true)
  {
      wasYPressed = true;
  }

  if (((master_controller.getDigital(okapi::ControllerDigital::Y)) == false) && wasYPressed)
  {
    wasYPressed = false;
    Y_pressed = !Y_pressed;

    if(Y_pressed == true)
    {
      tilter.set_value(true);
    }
    if(Y_pressed == false)
    {
      tilter.set_value(false);
    }
  }
  ////////////////////////////////////////////////////////////////////////////////////////////
}

void button_utilities_task()
{
  //uses the built-in pros task creator to start a task
  pros::Task button_initialize (button_utilities, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Button button_utilities Task");
}

void modified_opcontrol()
{
  while (true)
  {
    arcade_controls();
    intake_controls();
    arm_controls();
    button_utilities_task();
    pros::delay(10);
  }
}
