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


	drive_train.SetBrakeMode(okapi::AbstractMotor::brakeMode::brake);
	move_arm(-50, 100);
	lower_lifter();
	pros::delay(750);
	curve_drive_to_point(0.0_in, -3_in, 50, 15, 0.0208, -0.00416, 50, 0.004, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	raise_lifter();
	open_claw();
	curve_drive_to_point(52_in, -8_in, 100, 30, 0.024, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	// if(yellow_area > 25){
	// 	vision_drive(YELLOW_GOAL, 100, 50, 13, 0.001);
	// }
	close_claw();
	move_arm(720, 100);
	async_intake(600);
	curve_drive_to_point(86_in, -48_in, 100, 30, 0.022, -0.00416, 200, 0.0087, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	async_curve_drive_to_point(100_in, -48_in, 100, 30, 0.022, -0.00416, 200, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	pros::delay(750);
	move_arm(520, 100);
	wait_for_drive_complete_2();
	pros::delay(250);
	// gyro_turn_to(90_deg, 100, 10.0, 0.002, 0.0, 0.0, 2);
	open_claw();
	pros::delay(500);
	// gyro_turn_to(225_deg, 100, 10.0, 0.002, 0.0, 0.0, 2);
	curve_drive_to_point(92_in, -45_in, 100, 30, 0.022, -0.00416, 50, 0.005, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	async_drive_through_point(54_in, -62_in, 100, 25, 0.03, 0.00, 100, 0.015, false, 5, 0.5, 2);
	// drive_train.AutonomousArcadeDrive(0.0, 0.0);
	lower_lifter();
	pros::delay(750);
	stop_intake();
	async_curve_drive_to_point(24_in, -72_in, 100, 30, 0.022, -0.00416, 200, 0.0085, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	wait_for_drive_complete_2();
	// gyro_turn_to(360_deg, 100, 12.0, 0.003, 0.0, 0.0, 2);
	async_curve_drive_to_point(23_in, -94_in, 100, 30, 0.022, -0.00416, 200, 0.0087, 0.008, 200, 0, true, false, true, NONE, 5, 0.00001, 2, 28);
	async_intake(-600);
	wait_for_drive_complete_2();
	stop_intake();
	raise_lifter();
	async_intake(600);
	// drive_through_point(29_in, -83_in, 100, 25, 0.03, 0.00, 100, 0.01, false, 5, 0.5, 2);
	curve_drive_to_point(24_in, -80_in, 100, 30, 0.01, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 24);
	async_curve_drive_to_point(54_in, -80_in, 100, 30, 0.01, -0.00416, 100, 0.012, 0.008, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 24);
	pros::delay(500);
	move_arm(-50, 100);
	pros::delay(1000);
	stop_intake();
	wait_for_drive_complete_2();
	// if(yellow_area > 20){
	// 	vision_drive(YELLOW_GOAL, 100, 40, 12, 0.001);
	// }
	close_claw();
	move_arm(620, 100);
	async_intake(600);
	drive_through_point(75_in, -66_in, 100, 25, 0.03, 0.00, 100, 0.015, false, 5, 0.5, 2);
	async_curve_drive_to_point(99_in, -50_in, 100, 30, 0.022, -0.00416, 200, 0.009, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	wait_for_drive_complete_2();
	move_arm(520, 100);
	// gyro_turn_to(430_deg, 100, 12.0, 0.003, 0.0, 0.0, 2);
	gyro_turn_to(448_deg, 100, 9.0, 0.003, 0.0, 0.0, 2);
	open_claw();
	async_curve_drive_to_point(80_in, -60_in, 100, 30, 0.022, -0.00416, 100, 0.012, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 4, 28);
	pros::delay(400);
	move_arm(-30, 100);
	wait_for_drive_complete_2();
	async_curve_drive_to_point(92_in, -36_in, 100, 30, 0.024, -0.00416, 200, 0.007, 0.015, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 22);
	pros::delay(350);
	lower_lifter();
	wait_for_drive_complete_2();
	// if(blue_area > 20){
	// 	vision_drive(BLUE_GOAL, 100, 40, 12, 0.001);
	// }
	close_claw();
	move_arm(620, 100);
	curve_drive_to_point(84_in, -45_in, 100, 30, 0.022, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	async_curve_drive_to_point(99_in, -43_in, 100, 30, 0.022, -0.00416, 200, 0.0065, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	wait_for_drive_complete_2();
	move_arm(520, 100);
	pros::delay(750);
	open_claw();
	async_curve_drive_to_point(91_in, -38_in, 100, 30, 0.024, -0.00416, 80, 0.005, 0.015, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 22);
	pros::delay(400);
	move_arm(-50, 100);
	wait_for_drive_complete_2();
	normalize_inertial();
	async_curve_drive_to_point(87_in, -62_in, 100, 30, 0.022, -0.00416, 200, 0.0065, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	pros::delay(500);
	stop_intake();
	wait_for_drive_complete_2();
	close_claw();
	move_arm(570, 100);
	async_intake(600);
	curve_drive_to_point(83_in, -45_in, 90, 30, 0.022, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	async_curve_drive_to_point(100.5_in, -43_in, 100, 30, 0.022, -0.00416, 200, 0.0065, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	wait_for_drive_complete_2();
	move_arm(520, 100);
	pros::delay(750);
	open_claw();
	async_curve_drive_to_point(84_in, -40_in, 100, 30, 0.022, -0.00416, 200, 0.0065, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	pros::delay(500);
	move_arm(-50, 100);
	wait_for_drive_complete_2();
	stop_intake();
	drive_through_point(94_in, -76_in, 90, 25, 0.02, 0.00, 100, 0.015, false, 5, 0.5, 2);
	async_curve_drive_to_point(110_in, -90_in, 90, 30, 0.02, -0.00416, 100, 0.005, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	wait_for_drive_complete_2();
	curve_drive_to_point(118.5_in, -75_in, 90, 30, 0.022, -0.00416, 200, 0.01, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	raise_lifter();
	async_intake(600);
	async_curve_drive_to_point(67_in, -39_in, 100, 30, 0.02, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	pros::delay(2000);
	stop_intake();
	wait_for_drive_complete_2();
	close_claw();
	move_arm(510, 100);
	async_intake(600);
	async_curve_drive_to_point(118_in, 7_in, 100, 30, 0.022, -0.00416, 100, 0.013, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	wait_for_drive_complete_2();
	async_curve_drive_to_point(125_in, -9.5_in, 100, 30, 0.022, -0.00416, 100, 0.009, 0.008, 200, 0, false, false, false, NONE, 5, 0.00001, 2, 28);
	pros::delay(750);
	stop_intake();
	move_arm(430, 100);
	wait_for_drive_complete_2();
	async_intake(600);
	open_claw();
	pros::delay(750);
	async_curve_drive_to_point(124_in, 9_in, 100, 30, 0.022, -0.00416, 100, 0.009, 0.008, 200, 0, false, false, true, NONE, 5, 0.00001, 2, 28);
	pros::delay(500);
	move_arm(-50, 100);
	wait_for_drive_complete_2();
	normalize_inertial();
	async_curve_drive_to_point(106_in, 17.5_in, 100, 30, 0.022, -0.00416, 100, 0.009, 0.008, 200, 0, true, false, false, NONE, 5, 0.00001, 2, 28);
	pros::delay(750);
	stop_intake();
	wait_for_drive_complete_2();
	close_claw();
	async_intake(600);
	async_curve_drive_to_point(30_in, -34_in, 100, 30, 0.022, -0.00416, 100, 0.01, 0.008, 200, 0, true, false, false, NONE, 15, 0.00001, 2, 28);
	move_arm(580, 100);
	wait_for_drive_complete_2();
	open_claw();
	async_curve_drive_to_point(40_in, -44_in, 100, 30, 0.022, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, true, NONE, 15, 0.00001, 2, 28);
	wait_for_drive_complete_2();
	async_curve_drive_to_point(34_in, -15_in, 100, 30, 0.022, -0.00416, 100, 0.01, 0.008, 200, 0, false, false, false, NONE, 15, 0.00001, 2, 28);
	wait_for_drive_complete_2();
	lower_lifter();




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
