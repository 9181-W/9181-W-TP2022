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

	// gyro_turn(3600_deg, 40, 10.0, 0.003, 0.0, 0.0, 2);
	// gyro_drive(61.375_in, 0.25, 20, 50, true, 0.07, 1.0);

	// move_arm(-50, 100);
	// open_claw();
	// stick.set_value(true);
	// async_gyro_drive(23_in, 1.0, 100, 100, true, 0.02, 0.04);//0.04
	// wait_for_drive_complete();
	// curve_drive_to_point(-6_in, 32_in, 100, 80, 0.025, -0.00416, 100, 0.008, 1.0, 100, 100, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// close_claw();

	// move_arm(-50, 100);
	// open_claw();
	// stick.set_value(false);
	// async_gyro_drive(18_in, 1.0, 100, 100, true, 0.02, 0.04);//0.04
	// wait_for_drive_complete();
	// stick.set_value(true);
	// curve_drive_to_point(0_in, 8_in, 100, 100, 0.025, -0.00416, 100, 0.008, 1.0, 100, 20, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// close_claw();


	///////////////////////////////////////////////////////////////////////////////////////////////////////AUTONOMOUS
	// pros::delay(1000);
	//
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	// open_claw();
	// lower_lifter();
	// pros::delay(200);
	// curve_drive_to_point(0_in, -3_in, 100, 25, 0.025, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// raise_lifter();
	// move_arm(360, 100);
	// async_intake(600, true);
	// prog_skills_1();
	// stop_intake();
	// // lower_lifter();
	// move_arm(-30, 100);
	// // curve_drive_to_point(35_in, -82_in, 30, 30, 0.035, -0.00416, 0, 0.005, 0.0065, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// curve_drive_to_point(28_in, -98_in, 100, 25, 0.035, -0.00416, 100, 0.008, 0.0065, 100, 20, true, false, true, NONE, 2, 0.00001, 2, 28, 1);
	// raise_lifter();
	// async_curve_drive_to_point(36_in, -80_in, 100, 25, 0.025, -0.00416, 100, 0.009, 0.006, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// pros::delay(350);
	// async_intake(600, false);
	// wait_for_drive_complete_2();
	// close_claw();
	// move_arm(560, 100);
	// prog_skills_2();
	// move_arm(375, 100);
	// pros::delay(750);
	// open_claw();
	// stop_intake();
	// move_arm(700, 100);
	// curve_drive_to_point(90_in, -40_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// gyro_turn_to(-89.5_deg, 100, 10.0, 0.003, 0.0, 0.0, 1);
	// async_intake(600, false);
	// curve_drive_to_line(78_in, -39_in, 100, 30, 0.025, -0.00416, 100, 0.003, false, 0.4);
	// stop_intake();
	// lower_flap();
	// pros::delay(750);
	// async_curve_drive_to_point(80_in, -40_in, 100, 25, 0.025, -0.00416, 50, 0.001, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// pros::delay(500);
	// move_arm(-30, 100);
	// wait_for_drive_complete_2();
	// pros::delay(750);
	// curve_drive_to_line(72_in, -41_in, 100, 30, 0.022, -0.00416, 100, 0.01, false, 0.4);
	// // curve_drive_to_point(73_in, -39_in, 100, 25, 0.025, -0.00416, 50, 0.005, 0.006, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// close_claw();
	// pros::delay(250);
	// move_arm(500, 100);
	// gyro_turn_to(100_deg, 70, 10.0, 0.003, 0.0, 0.0, 2);
	// async_intake(600, false);
	// // curve_drive_to_line(98.5_in, -42_in, 100, 25, 0.022, -0.00416, 100, 0.01, false, 0.2);
	// curve_drive_to_point(102_in, -45_in, 100, 25, 0.025, -0.00416, 60, 0.008, 0.006, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// // prog_skills_3();
	// move_arm(350, 100);
	// pros::delay(750);
	// open_claw();
	// normalize_inertial();
	// async_curve_drive_to_point(94_in, -36_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// pros::delay(300);
	// move_arm(400, 100);
	// pros::delay(400);
	// move_arm(-50, 60);
	// wait_for_drive_complete_2();
	// curve_drive_to_point(68_in, -12_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// close_claw();
	// stop_intake();
	// move_arm(450, 100);
	// gyro_turn_to(160_deg, 100, 12.0, 0.004, 0.0, 0.0, 100);
	// prog_skills_3();
	// // async_curve_drive_to_point(99_in, -36_in, 100, 25, 0.025, -0.00416, 60, 0.008, 0.006, 200, 0, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// // pros::delay(2000);
	// // lower_lifter();
	// // wait_for_drive_complete_2();
	// open_claw();
	// async_curve_drive_to_point(92_in, -36_in, 100, 25, 0.025, -0.00416, 50, 0.001, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(83_in, -35_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// // pros::delay(800);
	// move_arm(-50, 100);
	// wait_for_drive_complete_2();
	// close_claw();
	// move_arm(500, 100);
	// curve_drive_to_point(99_in, -32_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// open_claw();
	// curve_drive_to_point(90_in, -34_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// async_intake(600, false);
	// curve_drive_to_point(93_in, -4_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// stop_intake();
	// curve_drive_to_point(94_in, 12_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, true, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// raise_lifter();
	// async_intake(600, false);
	// move_arm(-50, 100);
	// curve_drive_to_point(69_in, -56_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// close_claw();



	/////////////////////////////////////////////////////////////////////////////////

	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// // stick.set_value(false);
	// lower_flap();
	// async_gyro_drive(34_in, 1.0, 100, 95, true, 0.02, 0.1);//0.04
	// wait_for_drive_complete();
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	// close_claw();
	// lower_lifter();
	// right_side_rush_backup();
	// raise_lifter();
	// raise_flap();
	// move_arm(500, 100);
	// async_curve_drive_to_point(10_in, 50_in, 100, 20, 0.025, -0.00416, 60, 0.005, 0.006, 80, 10, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// pros::delay(300);
	// async_intake(600, false);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(24_in, 56_in, 100, 20, 0.025, -0.00416, 100, 0.007, 0.006, 60, 9, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(0_in, 10_in, 100, 20, 0.025, -0.00416, 100, 0.012, 0.007, 100, 10, true, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// pros::delay(750);
	// move_arm(0, 100);
	// wait_for_drive_complete_2();
	// lower_lifter();
	// gyro_drive(2_in, 0.09, 100, 15, true, 0.02, 0.04);//0.04
	// // curve_drive_to_point(-3_in, 12_in, 100, 20, 0.025, -0.00416, 100, 0.012, 0.007, 100, 10, false, false, false, NONE, 3, 0.00001, 2, 28, 1);

	///////////////////////////////////////////////////////////////////////////////////
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// stick.set_value(true);
	// async_gyro_drive(37_in, 1.0, 100, 100, true, 0.02, 0.04);//0.04
	// wait_for_drive_complete();
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	// close_claw();
	// lower_lifter();
	// curve_drive_to_point(0_in, 10_in, 100, 70, 0.025, -0.00416, 100, 0.015, 0.008, 100, 20, false, false, true, NONE, 3, 0.00001, 2, 28, 1);

	/////////////////////////////////////////////////////////////////////////////////////

	drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	open_claw();
	move_arm(-50, 100);
	lower_flap();
	curve_drive_to_point(-30_in, 48_in, 100, 70, 0.025, -0.00416, 100, 0.008, 0.007, 60, 9, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	wait_for_drive_complete();
	drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	close_claw();
	lower_lifter();
	right_side_rush_backup();
	raise_lifter();
	move_arm(475, 100);
	async_curve_drive_to_point(10_in, 50_in, 100, 20, 0.025, -0.00416, 60, 0.005, 0.006, 80, 10, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	pros::delay(300);
	async_intake(600, false);
	wait_for_drive_complete_2();
	async_curve_drive_to_point(24_in, 56_in, 100, 20, 0.025, -0.00416, 100, 0.007, 0.006, 60, 9, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	wait_for_drive_complete_2();
	async_curve_drive_to_point(0_in, 10_in, 100, 20, 0.025, -0.00416, 100, 0.012, 0.007, 100, 10, true, false, true, NONE, 3, 0.00001, 2, 28, 1);
	pros::delay(750);
	move_arm(0, 100);
	wait_for_drive_complete_2();
	lower_lifter();
	gyro_drive(2_in, 0.09, 100, 15, true, 0.02, 0.04);//0.04

	////////////////////////////////////////////////////////////////////////////////////////

	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// stick.set_value(false);
	// curve_drive_to_point(-16_in, 34_in, 100, 70, 0.025, -0.00416, 100, 0.015, 0.008, 100, 20, false, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// wait_for_drive_complete();
	// pros::delay(3000);
	// curve_drive_to_point(-66_in, 50_in, 100, 70, 0.025, -0.00416, 100, 0.015, 0.008, 100, 20, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// close_claw();
	// curve_drive_to_point(-10_in, 28_in, 100, 70, 0.025, -0.00416, 100, 0.015, 0.008, 100, 20, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// lower_lifter();



	// curve_drive_to_point(16.5_in, 19_in, 100, 70, 0.025, -0.00416, 100, 0.015, 0.008, 100, 20, false, false, true, NONE, 3, 0.00001, 2, 28, 1);
	// raise_lifter();
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);



	// test_points();
	// u_shape();
	// gyro_turn_to(0_deg, 40, 10.0, 0.003, 0.0, 0.0, 2);
	// curve_drive_to_point(24_in, 36_in, 100, 10, 0.0208, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28);

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
