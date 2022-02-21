#include "okapi/api.hpp"
#include "drive_train.hpp"

#define INTAKE_PORT 14
#define ARM_PORT 17
#define PNEUMATIC_CLAW_PORT 'A'
#define PNEUMATIC_FLAP_PORT 'D'
#define TILTER_PORT 'B'
#define BACK_CLAW_PORT 'C'
// #define OPTICAL_SHAFT_ENCODER_RIGHT_TOP 'A'
// #define OPTICAL_SHAFT_ENCODER_RIGHT_BOTTOM 'B'
// #define OPTICAL_SHAFT_ENCODER_MIDDLE_TOP 'C'
// #define OPTICAL_SHAFT_ENCODER_MIDDLE_BOTTOM 'D'
#define HORIZONTAL_ROTATION_SENSOR 20
#define VERTICAL_ROTATION_SENSOR 15

extern void modified_initialize();

extern DriveTrain drive_train;
extern okapi::Controller master_controller;
extern pros::Rotation* shaft_enc_r;
extern pros::Rotation* shaft_enc_m;
extern okapi::Motor intake_mtr;
extern okapi::Motor arm_mtr;
extern pros::ADIDigitalOut pneumatic_claw;
extern pros::ADIDigitalOut pneumatic_flap;
extern pros::ADIDigitalOut back_claw;
extern pros::ADIDigitalOut tilter;
