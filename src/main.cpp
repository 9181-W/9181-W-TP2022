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
#include "inertial.hpp"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
	modified_initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 //curve_drive_to_point(0.0_in, 48_in, 50, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, false, 5, 0.00001, 2);
void autonomous()
{

	////////////////////////////////////////////////////Juke Auto --slot 7/////////////////////////////////////////////////////

	// open_claw();
	// move_arm(-50, 100);
	// curve_drive_to_point(-29_in, 48_in, 100, 30, 0.03, 0.01, 100, 0.015, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// close_claw();
	// curve_drive_to_point(0_in, 10_in, 100, 30, 0.03, 0.01, 100, 0.012, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////MIDDLE RUSH --slot 6///////////////////////////////////////////////////

	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// async_gyro_drive(51_in, 0.5, 75, 50, true, 0.07, 1.0);//0.04
	// // pros::delay(700);
	// // lower_flap();
	// wait_for_drive_complete();
	// close_claw();
	// pros::delay(350);
	// lower_lifter();
	// async_curve_drive_to_point(21_in, 12_in, 100, 30, 0.03, 0.01, 100, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// pros::delay(750);
	// move_arm(100, 100);
	// wait_for_drive_complete_2();
	// raise_lifter();
	// async_intake(600);
	// curve_drive_to_point(15_in, 10_in, 100, 30, 0.03, 0.01, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// pros::delay(2000);
	// lower_lifter();
	// stop_intake();




	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	////////////////////////////////////////////////////DOUBLE RUSH --slot 5////////////////////////////////////////////////////////

	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// //void gyro_drive(QLength distance, double drive_kp, double max_speed, double min_speed, bool drive_straight, double drive_straight_kp, double epsilon);
	// async_gyro_drive(36.75_in, 1.0, 100, 100, true, 0.04, 1.0);//0.04
	// pros::delay(350);
	// lower_flap();
	// wait_for_drive_complete();
	// close_claw();
	// pros::delay(250);
	// curve_drive_to_point(1_in, 18_in, 100, 30, 0.03, 0.01, 100, 0.008, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// raise_flap();
	// move_arm(60, 100);
	// lower_lifter();
	// curve_drive_to_point(-34_in, 48_in, 100, 30, 0.03, 0.01, 100, 0.008, 0.008, 200, 0, true, false, true, NONE, 5, 0.00001, 2, 28);
	// raise_lifter();
	// curve_drive_to_point(1_in, 12_in, 100, 30, 0.03, 0.01, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	////////////////////////////////////////////////////LEFT SIDE MOGO RUSH --slot 3////////////////////////////////////////////////

	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// //void gyro_drive(QLength distance, double drive_kp, double max_speed, double min_speed, bool drive_straight, double drive_straight_kp, double epsilon);
	// // async_gyro_drive(43_in, 0.002, 90, 80, true, 0.08, 1.0);
	// async_gyro_drive(39_in, 1.0, 100, 100, true, 0.06, 1.0);//0.04
	// pros::delay(500);
	// lower_flap();
	// wait_for_drive_complete();
	// close_claw();
	// pros::delay(350);
	// async_curve_drive_to_point(-8_in, 0_in, 100, 30, 0.03, 0.01, 100, 0.008, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// pros::delay(750);
	// // move_arm(580, 100);
	// raise_flap();
	// wait_for_drive_complete_2();
	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// lower_lifter();
	// // move_arm(400, 100);
	// // gyro_turn_to(-100_deg, 100, 10.0, 0.003, 0.0, 0.0, 2);
	// curve_drive_to_point(5.5_in, -1_in, 40, 30, 0.03, 0.01, 50, 0.008, 0.008, 200, 0, true, false, true, NONE, 1, 0.00001, 2, 28);
	// raise_lifter();
	// move_arm(400, 100);
	// async_intake(600);
	// curve_drive_to_point(0_in, 24_in, 100, 30, 0.03, 0.01, 100, 0.002, 0.01, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 28);
	// curve_drive_to_point(45_in, 41_in, 30, 30, 0.03, 0.01, 100, 0.008, 0.01, 200, 0, true, false, false, NONE, 2, 0.00001, 2, 28);
	// lower_lifter();
	// move_arm(0, 100);
	// curve_drive_to_point(51_in, 42_in, 100, 30, 0.03, 0.01, 100, 0.002, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// stop_intake();



	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	////////////////////////////////////////////////////Match Auto - Carry --slot 8/////////////////////////////////////////////////////////

	// lower_flap();
	// pros::delay(300);
	// raise_flap();
	// curve_drive_to_point(-20_in, 0_in, 100, 35, 0.04, -0.00416, 100, 0.01, 0.008, 200, 0, true, false, true, NONE, 5, 0.00001, 2, 28);
	// curve_drive_to_point(-25_in, 70_in, 100, 35, 0.04, -0.00416, 100, 0.008, 0.008, 200, 0, true, false, true, NONE, 5, 0.00001, 2, 28);
	// open_claw();
	// move_arm(-50, 100);
	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	// curve_drive_to_point(-52_in, 52_in, 100, 35, 0.028, -0.00416, 100, 0.008, 0.008, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 28);
	// close_claw();
	// lower_lifter();
	// move_arm(80, 100);
	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// curve_drive_to_point(-26_in, 98_in, 100, 35, 0.023, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// raise_lifter();
	// move_arm(400, 100);
	// curve_drive_to_point(-26_in, 96_in, 100, 35, 0.04, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// async_intake(600);
	// curve_drive_to_point(-55_in, 92_in, 100, 35, 0.03, -0.00416, 100, 0.002, 0.006, 100, 0, true, false, false, NONE, 5, 0.00001, 2, 28);
	// async_curve_drive_to_point(-32_in, 86_in, 100, 35, 0.035, -0.00416, 100, 0.002, 0.008, 200, 0, true, false, true, NONE, 5, 0.00001, 2, 28);
	// pros::delay(1250);
	// move_arm(80, 100);
	// wait_for_drive_complete_2();
	// lower_lifter();
	// stop_intake();
	// curve_drive_to_point(-31_in, 96_in, 100, 35, 0.03, -0.00416, 100, 0.002, 0.008, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 28);

	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// pros::delay(750);
	// curve_drive_to_point(0.0_in, -3_in, 50, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// curve_drive_to_point(24_in, 0_in, 100, 35, 0.0208, -0.00416, 100, 0.012, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// async_intake(600);
	// move_arm(340, 100);
	// curve_drive_to_point(20_in, -73_in, 85, 30, 0.0208, -0.00416, 100, 0.008, 0.008, 200, 0, true, false, true, NONE, 5, 0.00001, 2, 28);
	// lower_lifter();
	// stop_intake();
	// gyro_turn_to(-280_deg, 100, 10.0, 0.003, 0.0, 0.0, 2);
	// curve_drive_to_point(26_in, -96_in, 100, 30, 0.0208, -0.00416, 100, 0.008, 0.008, 200, 0, true, false, true, NONE, 3, 0.00001, 2, 28);
	// raise_lifter();
	// async_intake(600);
	// async_curve_drive_to_point(58_in, -42_in, 100, 30, 0.0208, -0.00416, 100, 0.008, 0.005, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 28);
	// pros::delay(500);
	// move_arm(-50, 60);
	// wait_for_drive_complete_2();
	// close_claw();
	// move_arm(200, 100);
	// curve_drive_to_point(30_in, -15_in, 100, 40, 0.04, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// lower_lifter();




	////////////////////////////////////////////////////Match Auto Race For Yellow --slot 2////////////////////////////////////////////////

// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
// 	open_claw();
// 	move_arm(-50, 100);
// 	//void gyro_drive(QLength distance, double drive_kp, double max_speed, double min_speed, bool drive_straight, double drive_straight_kp, double epsilon);
// 	async_gyro_drive(36.75_in, 1.0, 100, 100, true, 0.04, 1.0);//0.04
// 	pros::delay(275);
// 	lower_flap();
// 	wait_for_drive_complete();
// 	// curve_drive_to_point(0_in, 40_in, 100, 100, 0.03, -0.05, 100, 0.04, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
// 	close_claw();
// 	pros::delay(250);
//
// 	curve_drive_to_point(1_in, 19_in, 100, 30, 0.03, 0.01, 100, 0.008, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
// 	raise_flap();
// 	move_arm(640, 100);
// 	lower_lifter();
// 	curve_drive_to_point(18_in, 19_in, 100, 30, 0.03, 0.01, 100, 0.009, 0.01, 200, 0, true, false, true, NONE, 5, 0.001, 2, 28);//cutout changed
// 	raise_lifter();
// 	curve_drive_to_point(14_in, 19_in, 100, 30, 0.03, 0.01, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
// 	async_intake(600);
// 	// drive_through_point(12_in, 52_in, 30, 10, 0.02, 0.00, 100, 0.007, false, 5, 0.5, 2);
// 	curve_drive_to_point(11_in, 53_in, 30, 10, 0.03, 0.01, 100, 0.008, 0.007, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
// 	curve_drive_to_point(23_in, 55_in, 30, 10, 0.03, 0.01, 100, 0.008, 0.007, 200, 0, true, false, false, NONE, 5, 0.001, 2, 28);//cutout changed
// 	async_curve_drive_to_point(-4_in, 16_in, 100, 30, 0.03, 0.01, 100, 0.008, 0.01, 200, 0, true, false, true, NONE, 5, 0.00001, 2, 28);
// 	pros::delay(1250);
// 	move_arm(0, 100);
// 	stop_intake();
// 	wait_for_drive_complete_2();
// 	lower_lifter();
// 	curve_drive_to_point(5_in, 22_in, 30, 10, 0.03, 0.01, 100, 0.008, 0.01, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
// 	pros::lcd::print(2,"EXIT");

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	// open_claw();
	// curve_drive_to_point(0.0_in, 12_in, 50, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, false, YELLOW_GOAL, 5, 0.00001, 2, 28);
	// vision_drive(YELLOW_GOAL, 100, 40, 13, 0.001);
	// close_claw();
	// pros::delay(10000);
	// curve_drive_to_point(0.0_in, 24_in, 50, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, false, YELLOW_GOAL, 5, 0.00001, 2, 28);
	// vision_drive(YELLOW_GOAL, 100, 40, 13, 0.001);

	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// raise_lifter();
	// close_claw();
	// move_arm(-520, 100);
	// pros::delay(1500);
	// curve_drive_to_point(0_in, 41_in, 100, 100, 0.1, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 11, 28);
	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);


	/*
	curve_drive_to_point(24_in, 36_in, 100, 30, 0.024, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
  gyro_turn_to(180_deg, 30, 10.0, 0.003, 0.0, 0.0, 2);
	pros::delay(1000);
  gyro_turn_to(270_deg, 30, 10.0, 0.003, 0.0, 0.0, 2);
	curve_drive_to_point(-24_in, 36_in, 100, 30, 0.024, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
  gyro_turn_to(400_deg, 30, 10.0, 0.003, 0.0, 0.0, 2);
	normalize_inertial();
	pros::delay(1000);
	curve_drive_to_point(0_in, 0_in, 100, 30, 0.024, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	gyro_turn_to(180_deg, 30, 10.0, 0.003, 0.0, 0.0, 2);
	pros::delay(1000);
	gyro_turn_to(90_deg, 30, 10.0, 0.003, 0.0, 0.0, 2);
	pros::delay(1000);
	gyro_turn_to(0_deg, 30, 10.0, 0.003, 0.0, 0.0, 2);
	*/
	// gyro_drive(-36_in, 50);
	// gyro_turn_to(0_deg, 40, 10.0, 0.003, 0.0, 0.0, 2);

	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	//
	// gyro_turn_to(3600_deg, 100, 9.0, 0.0025, 0.0, 0.0, 1);
	// // pros::delay(10000);
	// // normalize_inertial();
	// // curve_drive_to_point(0_in, 27_in, 50, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// // gyro_turn_to(0_deg, 100, 9.0, 0.0025, 0.0, 0.0, 1);

	/////////////////////////////////////////////////////////////////////PROGRAMMING SKILLS//////////////////////////////////////////////////////////////////////

	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::coast);
	// move_arm(-50, 100);
	// lower_lifter();
	// pros::delay(750);
	// curve_drive_to_point(0.0_in, -3_in, 50, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// raise_lifter();
	// open_claw();
	// curve_drive_to_point(54_in, -3_in, 100, 30, 0.024, -0.00416, 200, 0.0095, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// // if(yellow_area > 25){
	// // 	vision_drive(YELLOW_GOAL, 100, 50, 13, 0.001);
	// // }
	// close_claw();
	// move_arm(720, 100);
	// async_curve_drive_to_point(86_in, -46_in, 100, 30, 0.022, -0.00416, 200, 0.0087, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// 	pros::delay(500);
	// 	async_intake(600);
	// wait_for_drive_complete_2();
	// //moved left DANGER
	// async_curve_drive_to_point(100_in, -45_in, 100, 30, 0.022, -0.00416, 200, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// 	pros::delay(750);
	// 	move_arm(480, 100);
	// wait_for_drive_complete_2();
	// pros::delay(600);
	// // gyro_turn_to(90_deg, 100, 10.0, 0.002, 0.0, 0.0, 2);
	// open_claw();
	// pros::delay(500);
	// move_arm(520, 100);
	// // gyro_turn_to(225_deg, 100, 10.0, 0.002, 0.0, 0.0, 2);
	// curve_drive_to_point(92_in, -41_in, 100, 30, 0.022, -0.00416, 50, 0.005, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// //Driving Between
	// // async_drive_through_point(54_in, -62_in, 100, 25, 0.03, 0.00, 100, 0.015, false, 5, 0.5, 2);
	// // async_curve_drive_to_point(54_in, -61_in, 100, 30, 0.03, -0.00416, 100, 0.015, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28)//red side start old
	// // async_curve_drive_to_point(54_in, -62_in, 100, 30, 0.03, -0.00416, 100, 0.015, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);//red side start
	// // async_curve_drive_to_point(54_in, -62.5_in, 100, 30, 0.03, -0.00416, 100, 0.015, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);//initial working
	// async_curve_drive_to_point(54_in, -64_in, 100, 30, 0.03, -0.00416, 100, 0.015, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);//initial working
	//
	// // async_curve_drive_to_point(54_in, -62.5_in, 100, 30, 0.03, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);//upstairs start
	//
	// lower_lifter();
	// pros::delay(750);
	// stop_intake();
	// wait_for_drive_complete_2();
	//
	// async_curve_drive_to_point(28_in, -76_in, 100, 30, 0.04, -0.00416, 100, 0.015, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// wait_for_drive_complete_2();
	// // normalize_inertial();
	// // gyro_turn_to(360_deg, 100, 12.0, 0.003, 0.0, 0.0, 2);
	// async_curve_drive_to_point(24_in, -95_in, 100, 30, 0.03, -0.00416, 100, 0.012, 0.008, 200, 0, true, false, true, NONE, 5, 0.00001, 2, 28);
	// async_intake(-600);
	// wait_for_drive_complete_2();
	// stop_intake();
	// raise_lifter();
	// async_intake(600);
	// // drive_through_point(29_in, -83_in, 100, 25, 0.03, 0.00, 100, 0.01, false, 5, 0.5, 2);
	// curve_drive_to_point(24_in, -80_in, 100, 30, 0.01, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 24);
	// async_curve_drive_to_point(57_in, -78.5_in, 100, 30, 0.01, -0.00416, 100, 0.012, 0.008, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 24);
	// pros::delay(500);
	// move_arm(-50, 100);
	// pros::delay(1000);
	// stop_intake();
	// wait_for_drive_complete_2();
	// // if(yellow_area > 20){
	// // 	vision_drive(YELLOW_GOAL, 100, 40, 12, 0.001);
	// // }
	// close_claw();
	// move_arm(580, 100);
	// async_intake(600);
	// drive_through_point(75_in, -66_in, 100, 25, 0.03, 0.00, 100, 0.015, false, 5, 0.5, 2);
	// async_curve_drive_to_point(101_in, -55_in, 100, 30, 0.022, -0.00416, 200, 0.009, 0.008, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 28);//changed by -1 on y
	// wait_for_drive_complete_2();
	// move_arm(520, 100);
	// // gyro_turn_to(430_deg, 100, 12.0, 0.003, 0.0, 0.0, 2);
	// // gyro_turn_to(448_deg, 100, 9.0, 0.003, 0.0, 0.0, 2);
	// open_claw();
	// async_curve_drive_to_point(80_in, -60_in, 100, 30, 0.022, -0.00416, 100, 0.012, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 4, 28);
	// pros::delay(400);
	// move_arm(-30, 100);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(92_in, -36_in, 100, 30, 0.024, -0.00416, 200, 0.007, 0.015, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 22);
	// pros::delay(350);
	// lower_lifter();
	// pros::delay(350);
	// stop_intake();
	// wait_for_drive_complete_2();
	// async_intake(600);
	// // if(blue_area > 20){
	// // 	vision_drive(BLUE_GOAL, 100, 40, 12, 0.001);
	// // }
	// close_claw();
	// move_arm(570, 100);
	// curve_drive_to_point(84_in, -41_in, 100, 30, 0.022, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// async_curve_drive_to_point(100_in, -38_in, 100, 30, 0.024, -0.00416, 200, 0.008, 0.008, 200, 0, true, false, false, NONE, 5, 0.001, 2, 28);//cutout changed
	// wait_for_drive_complete_2();
	// // move_arm(510, 100);
	// pros::delay(750);
	// open_claw();
	// async_curve_drive_to_point(83_in, -31_in, 100, 30, 0.024, -0.00416, 80, 0.005, 0.015, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 22);
	// pros::delay(400);
	// move_arm(-50, 100);
	// wait_for_drive_complete_2();
	// normalize_inertial();
	// //shorten y for time.
	// async_curve_drive_to_point(85_in, -62_in, 100, 30, 0.022, -0.00416, 200, 0.0065, 0.009, 200, 5, true, false, false, NONE, 5, 0.00001, 2, 28);
	// pros::delay(500);
	// stop_intake();
	// wait_for_drive_complete_2();
	// close_claw();
	// move_arm(570, 100);
	// async_intake(600);
	// curve_drive_to_point(83_in, -44_in, 90, 30, 0.022, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// async_curve_drive_to_point(100_in, -44_in, 100, 30, 0.022, -0.00416, 200, 0.0065, 0.008, 200, 0, true, false, false, NONE, 5, 0.001, 2, 28);//cutout changed
	// wait_for_drive_complete_2();
	// move_arm(510, 100);
	// pros::delay(750);
	// open_claw();
	// async_curve_drive_to_point(84_in, -40_in, 100, 30, 0.022, -0.00416, 200, 0.0065, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// wait_for_drive_complete_2();
	// stop_intake();
	// drive_through_point(94_in, -76_in, 90, 25, 0.02, 0.00, 100, 0.015, false, 5, 0.5, 2);
	// async_curve_drive_to_point(110_in, -92_in, 90, 30, 0.02, -0.00416, 100, 0.005, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// wait_for_drive_complete_2();
	// //mogo under platfrom
	// // curve_drive_to_point(117.5_in, -78_in, 70, 30, 0.022, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);//red side start
	// curve_drive_to_point(117.5_in, -79.5_in, 70, 30, 0.022, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);//blue side start
	// raise_lifter();
	// move_arm(-50, 50);
	// async_intake(600);
	// //big yellow
	// async_curve_drive_to_point(61_in, -37_in, 100, 30, 0.02, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// pros::delay(2500);
	// stop_intake();
	// wait_for_drive_complete_2();
	// close_claw();
	// move_arm(550, 100);
	// async_intake(600);
	// async_curve_drive_to_point(110_in, 3_in, 100, 30, 0.022, -0.00416, 100, 0.013, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(117_in, -10_in, 100, 30, 0.022, -0.00416, 100, 0.009, 0.008, 200, 0, false, false, false, NONE, 5, 0.001, 2, 28);
	// pros::delay(400);
	// stop_intake();
	// move_arm(415, 100);
	// wait_for_drive_complete_2();
	// open_claw();
	// pros::delay(750);
	// async_curve_drive_to_point(117_in, 14_in, 100, 30, 0.022, -0.00416, 100, 0.009, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	// pros::delay(500);
	// async_intake(600);
	// move_arm(-50, 100);
	// wait_for_drive_complete_2();
	// normalize_inertial();
	// async_curve_drive_to_point(100_in, 18.5_in, 100, 30, 0.022, -0.00416, 100, 0.009, 0.008, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 28);
	// pros::delay(750);
	// stop_intake();
	// wait_for_drive_complete_2();
	// close_claw();
	// async_intake(600);
	//
  // //mogo stack code
	// async_curve_drive_to_point(16_in, -34_in, 100, 30, 0.022, -0.00416, 100, 0.015, 0.008, 200, 0, false, false, false, NONE, 15, 0.001, 2, 28);//changed cutout
	// move_arm(580, 100);
	// wait_for_drive_complete_2();
	// open_claw();
	// async_curve_drive_to_point(20_in, -44_in, 100, 30, 0.022, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, true, NONE, 15, 0.00001, 2, 28);
	// wait_for_drive_complete_2();
	// lower_lifter();
	// async_curve_drive_to_point(30_in, -15_in, 100, 30, 0.022, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, false, NONE, 15, 0.00001, 2, 28);
	// wait_for_drive_complete_2();


	//parking code
	// move_arm(600, 100);
	// curve_drive_to_point(12_in, 6_in, 100, 30, 0.022, -0.00416, 100, 0.011, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// async_curve_drive_to_point(0_in, 1_in, 100, 30, 0.022, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// pros::delay(1250);
	// move_arm(-50, 100);
	// wait_for_drive_complete_2();
	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// async_curve_drive_to_point(0_in, -43.5_in, 100, 100, 0.1, -0.00416, 100, 0.01, 0.008, 200, 0, true, true, false, NONE, 5, 0.00001, 11, 28);
	// wait_for_drive_complete_2();




	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




	// async_curve_drive_to_point(14_in, -3_in, 100, 30, 0.022, -0.00416, 100, 0.011, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// pros::delay(750);
	// move_arm(-50, 100);
	// wait_for_drive_complete_2();
	// pros::delay(1000);
	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// async_curve_drive_to_point(14_in, -43.5_in, 100, 100, 0.1, -0.00416, 100, 0.01, 0.008, 200, 0, true, false, false, NONE, 5, 0.00001, 11, 28);
	// wait_for_drive_complete_2();






	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	// open_claw();
	// curve_drive_to_point(0_in, 36_in, 60, 40, 0.02, -0.004, 40, 0.004, 0.008, 200, 0, false, false, false, RED_GOAL, 5, 0.00001, 2);
	// vision_drive(RED_GOAL, 60, 40);
	// close_claw();
	// open_claw();
	// pros::delay(500);
	// vision_drive(RED_GOAL, 100.0, 30.0);
	// close_claw();




	// drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::hold);
	// open_claw();
	// curve_drive_to_point(0.0_in, 35_in, 100, 0, 0.08, -0.004, 40, 0.001, 0.008, 200, 0, false, false, false, 5, 0.00001, 2);
	// close_claw();
	// curve_drive_to_point(0.0_in, 5_in, 100, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, true, 5, 0.00001, 2);
	// lower_lifter();
	// move_arm(120, 100);
	// curve_drive_to_point(12_in, 13_in, 100, 30, 0.022, -0.00216, 50, 0.00, 0.007, 200, 0, true, false, true, 3, 0.00001, 2);
	// raise_lifter();
	// move_arm(400, 100);
	// async_intake(600);
	// curve_drive_to_point(8_in, 40_in, 100, 30, 0.022, -0.00216, 40, 0.004, 0.007, 200, 0, true, false, false, 7, 0.00001, 2);
	// curve_drive_to_point(8_in, 10_in, 100, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, true, 5, 0.00001, 2);
	// stop_intake();




	//gyro_drive(48_in, 50, true);
	// drive_train.MoveVelocity(0.5);
	// pros::delay(1000);
	// // drive_train.MoveVelocity(0.5);
	// // pros::delay(5000);
	// drive_train.MoveVelocity(0.0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	kill_all_tasks();
	modified_opcontrol();

	// DriveTrain drive_train({11, 12}, {-20, -19});
	//
	// okapi::Controller master_controller(ControllerId::master);
	//
	// while (true)
	// {
	// 	double leftY = master_controller.getAnalog(okapi::ControllerAnalog::leftY);
	//   double leftX = (master_controller.getAnalog(okapi::ControllerAnalog::leftX) * -1);
	//
	// 	drive_train.ArcadeDrive(leftY, leftX);
	// 	pros::delay(20);
	// }
}
