#include "drive_train.hpp"
#include "okapi/api.hpp"
#include "initialize.hpp"
#include "inertial.hpp"
#include "position_tracker.hpp"
#include "vision.hpp"

//creates the shaft encoders as empty objects
pros::Rotation* shaft_enc_r = NULL;
pros::Rotation* shaft_enc_m = NULL;

DriveTrain drive_train({10, -18, -19}, {-11, 12, 13});
// DriveTrain drive_train({11, 12}, {-19, -20});
//20 15

okapi::Controller master_controller(ControllerId::master);
okapi::Motor intake_mtr(INTAKE_PORT, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
okapi::Motor arm_mtr(ARM_PORT, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

pros::ADIDigitalOut pneumatic_claw(PNEUMATIC_CLAW_PORT);
pros::ADIDigitalOut back_claw(BACK_CLAW_PORT);
pros::ADIDigitalOut tilter(TILTER_PORT);

//creates a task to display the encoder values
void encoder_display_task(void* param)
{
  while(true)
  {
    pros::delay(10);

    //prints the data from the shaft encoders to the lcd screen
    // printf("raw_left: %5.1f\n", shaft_enc_l->get());
    // printf("raw_right: %5.1f\n", shaft_enc_r->get());
    // printf("raw_middle: %5.1f\n", shaft_enc_m->get());
    // pros::lcd::print(3, "raw_left: %5.1f", shaft_enc_l->get());
    // pros::lcd::print(4, "raw_right: %5.1f", shaft_enc_r->get());
    // pros::lcd::print(5, "raw_middle: %5.1f", shaft_enc_m->get());
  }
}

void modified_initialize()
{
  //inits. the lcd screen
  pros::lcd::initialize();
  //prints init. to the lcd
  pros::lcd::print(0, "initialize");

  //creates the shaft encoders as objects
  // shaft_enc_r = new ADIEncoder(OPTICAL_SHAFT_ENCODER_RIGHT_TOP, OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM, true);
  // shaft_enc_m = new ADIEncoder(OPTICAL_SHAFT_ENCODER_MIDDLE_TOP, OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM, false);

  shaft_enc_r = new pros::Rotation(VERTICAL_ROTATION_SENSOR);
  shaft_enc_m = new pros::Rotation(HORIZONTAL_ROTATION_SENSOR);
  shaft_enc_r->reset_position();
  shaft_enc_m->reset_position();

  //creates a task to display encoder values and make them continually drawable
  pros::Task encoder_display (encoder_display_task, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Encoder Display Task");

  inertial_initialize();

  tracker_initialize();

  vision_initialize();

  arm_mtr.setBrakeMode(okapi::AbstractMotor::brakeMode::hold);

}
