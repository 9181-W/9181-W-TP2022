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

	/////////////////////////////////////////////////////////////////AWP_CARRY/////////////////////////////////////////////////////////////////////////////////////////

	drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	open_claw();
	lower_lifter();
	pros::delay(200);
	awp_path_1();
	// curve_drive_to_point(-20_in, -11_in, 100, 25, 0.025, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1, 20);
	raise_lifter();
	async_intake(600, false);
	curve_drive_to_point(38_in, -47_in, 100, 25, 0.025, -0.00416, 100, 0.014, 0.008, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 1, 10);
	close_claw();
	move_arm(300, 100);
	async_curve_drive_to_point(85_in, -15_in, 100, 25, 0.025, -0.00416, 100, 0.015, 0.005, 100, 7, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 20);
	pros::delay(2750);
	lower_lifter();
	move_arm(-50, 100);
	wait_for_drive_complete_2();
	stop_intake();
	open_claw();
	async_curve_drive_to_point(69_in, 4_in, 100, 25, 0.025, -0.00416, 100, 0.015, 0.006, 100, 7, true, false, true, NONE, 3, 0.00001, 2, 28, 1, 20);
	wait_for_drive_complete_2();
	raise_lifter();


	/////////////////////////////////////////////////////////////////PROGRAMMING_SKILLS////////////////////////////////////////////////////////////////////////////////

	// driv`e_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	// open_claw();
	// lower_lifter();
	// pros::delay(200);
	// curve_drive_to_point(0_in, -3_in, 100, 25, 0.025, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);
	// raise_lifter();
	// // move_arm(360, 100);
	// async_intake(600, true);
	// async_curve_drive_to_point(20_in, 5.5_in, 100, 25, 0.025, -0.00416, 100, 0.008, 0.006, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// gyro_turn_to(177_deg, 100, 7.0, 0.0065, 0.0, 0.0, 3, false);
	// move_arm(-50, 100);
	// // async_curve_drive_to_point(22_in, -90_in, 100, 25, 0.025, -0.00416, 100, 0.01, 0.006, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 1, 5);
	// prog_skills_1();
	// wait_for_drive_complete_2();
	// close_claw();
	// stop_intake();
	//
	// // curve_drive_to_point(28_in, -98_in, 100, 25, 0.035, -0.00416, 100, 0.008, 0.0045, 80, 10, true, false, true, NONE, 2, 0.00001, 2, 28, 1, 30);
	// // raise_lifter();
	// move_arm(560, 100);
	// async_curve_drive_to_point(33_in, -76_in, 100, 35, 0.025, -0.00416, 100, 0.009, 0.006, 200, 0, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
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
	// curve_drive_to_point(90_in, -40_in, 100, 10, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1.5, 30);
	// gyro_turn_to(-89.5_deg, 100, 10.0, 0.006, 0.0, 0.0, 2, true);
	// async_intake(600, false);
	// // curve_drive_to_line(77.75_in, -40_in, 100, 30, 0.025, -0.00416, 100, 0.003, false, 0.4);//changed from -39
	// curve_drive_to_line(77.75_in, -39.5_in, 100, 30, 0.025, -0.00416, 100, 0.005, false, 0.4);//changed from -39
	// stop_intake();
	// lower_flap();
	// pros::delay(750);
	// async_curve_drive_to_point(80_in, -40_in, 100, 25, 0.025, -0.00416, 50, 0.001, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);
	// pros::delay(500);
	// move_arm(-30, 100);
	// wait_for_drive_complete_2();
	// pros::delay(750);
	// curve_drive_to_line(72_in, -41_in, 100, 30, 0.022, -0.00416, 100, 0.01, false, 0.4);
	// close_claw();
	// pros::delay(250);
	// move_arm(480, 100);
	// // gyro_turn_to(95_deg, 70, 10.0, 0.003, 0.0, 0.0, 2, true);
	// gyro_turn_to(120_deg, 70, 10.0, 0.005, 0.0, 0.0, 2, true);
	// async_intake(600, false);
	// // curve_drive_to_point(101.5_in, -45.5_in, 100, 25, 0.025, -0.00416, 60, 0.008, 0.006, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 2, 35);
	// curve_drive_to_point(101.5_in, -47.5_in, 100, 25, 0.025, -0.00416, 60, 0.008, 0.006, 200, 0, false, false, false, NONE, 3, 0.00001, 2, 28, 2, 20);
	// move_arm(335, 100);
	// pros::delay(750);
	// open_claw();
	// normalize_inertial();
	// async_curve_drive_to_point(93_in, -38_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 2, 30);
	// pros::delay(300);
	// move_arm(400, 100);
	// pros::delay(400);
	// move_arm(-50, 40);
	// wait_for_drive_complete_2();
	// curve_drive_to_point(68_in, -8_in, 100, 35, 0.025, -0.00416, 100, 0.008, 0.0065, 70, 0, true, false, false, NONE, 7, 0.00001, 2, 28, 2, 10);
	// close_claw();
	// stop_intake();
	// move_arm(475, 100);
	// // gyro_turn_to(160_deg, 100, 7.0, 0.006, 0.0, 0.0, 2, true);
	// gyro_turn_to(160_deg, 100, 7.0, 0.006, 0.0, 0.0, 3, true);
	// prog_skills_3();
	// open_claw();
	// normalize_inertial();
	// async_curve_drive_to_point(92_in, -38_in, 100, 25, 0.025, -0.00416, 70, 0.008, 0.006, 200, 0, false, false, true, NONE, 2, 0.00001, 2, 28, 3, 15);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(81.5_in, -36.25_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 15);
	// move_arm(-50, 100);
	// wait_for_drive_complete_2();
	// close_claw();
	// move_arm(475, 100);
	// curve_drive_to_point(98_in, -33_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 7, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);//-33
	// open_claw();
	// curve_drive_to_point(90_in, -34_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, false, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);
	// async_intake(600, false);
	// // curve_drive_to_point(93_in, -6_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// stop_intake();
	// curve_drive_to_point(95_in, 11_in, 100, 25, 0.025, -0.00416, 100, 0.005, 0.006, 200, 0, true, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);
	// raise_lifter();
	// async_intake(600, false);
	// move_arm(-50, 100);
	// normalize_inertial();
	// prog_skills_4();
	// close_claw();
	// move_arm(450, 100);
	// curve_drive_to_point(99_in, -58_in, 100, 25, 0.035, -0.00416, 100, 0.007, 0.0045, 40, 10, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// open_claw();
	// pros::delay(350);
	// async_curve_drive_to_point(95_in, -81_in, 100, 45, 0.035, -0.00416, 100, 0.007, 0.0045, 40, 10, true, false, true, NONE, 3, 0.001, 2, 28, 1, 30);
	// pros::delay(650);
	// move_arm(-50, 100);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(110.5_in, -77_in, 100, 35, 0.035, -0.00416, 100, 0.01, 0.0045, 40, 10, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// wait_for_drive_complete_2();
	// close_claw();
	// normalize_inertial();
	// async_curve_drive_to_point(103_in, -82_in, 100, 25, 0.05, -0.00416, 100, 0.01, 0.006, 60, 10, false, false, true, NONE, 3, 0.00001, 2, 28, 1, 20);
	// move_arm(50, 100);
	// wait_for_drive_complete_2();
	// async_intake(600, false);
	// async_curve_drive_to_point(6_in, -82_in, 100, 25, 0.035, -0.00416, 100, 0.01, 0.0045, 40, 10, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 10);
	// pros::delay(600);
	// move_arm(500, 100);
	// wait_for_drive_complete_2();
	// // normalize_inertial();
	// gyro_turn_to(-1_deg, 100, 8.0, 0.006, 0.0, 0.0, 3, true);
	// move_arm(-50, 100);
	// pros::delay(1000);
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// drive_to_pitch();
	// gyro_drive(21_in, 0.3, 100, 50, true, 0.02, 0.1);//0.04
	// lower_lifter();`

	/////////////////////////////////////////////////////////////MIDDLE_KNOCK_OVER////////////////////////////////////////////////////////////////

	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// async_gyro_drive(45_in, 1.0, 100, 100, true, 0.02, 0.04);//0.04
	// move_arm(300, 100);
	// lower_flap();
	// wait_for_drive_complete();
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);

	///////////////////////////////////////////////////////RIGHT_SIDE_MOGO_RUSH --SLOT 2///////////////////////////////////////////////////////////////////

	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// // stick.set_value(false);
	// lower_flap();
	// async_gyro_drive(34_in, 1.0, 100, 95, true, 0.02, 0.04);//0.04
	// 	// async_gyro_drive(34_in, 1.0, 100, 95, true, 0.02, 0.1);//0.04
	// wait_for_drive_complete();
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	// close_claw();
	// lower_lifter();
	// right_side_rush_backup();
	// raise_lifter();
	// raise_flap();
	// move_arm(500, 100);
	// async_curve_drive_to_point(10_in, 50_in, 100, 20, 0.025, -0.00416, 60, 0.005, 0.006, 80, 10, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// pros::delay(300);
	// async_intake(600, false);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(24_in, 56_in, 100, 20, 0.025, -0.00416, 100, 0.007, 0.006, 60, 9, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(0_in, 10_in, 100, 20, 0.025, -0.00416, 100, 0.012, 0.007, 100, 10, true, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);
	// pros::delay(750);
	// move_arm(0, 100);
	// wait_for_drive_complete_2();
	// lower_lifter();
	// gyro_drive(2_in, 0.09, 100, 15, true, 0.02, 0.04);//0.04
	// // curve_drive_to_point(-3_in, 12_in, 100, 20, 0.025, -0.00416, 100, 0.012, 0.007, 100, 10, false, false, false, NONE, 3, 0.00001, 2, 28, 1);

	////////////////////////////////////////////////////////////LEFT_SIDE_MOGO_RUSH --SLOT 3//////////////////////////////////////////////////////////////////////
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// async_gyro_drive(37_in, 1.0, 100, 100, true, 0.02, 0.04);//0.04
	// lower_flap();
	// wait_for_drive_complete();
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	// close_claw();
	// lower_lifter();
	// curve_drive_to_point(0_in, 10_in, 100, 70, 0.025, -0.00416, 100, 0.015, 0.008, 100, 20, false, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);

	//////////////////////////////////////////////////////////////MIDDLE_JUKE --SLOT 4//////////////////////////////////////////////////////////////////

	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// lower_flap();
	// curve_drive_to_point(-30_in, 48_in, 100, 70, 0.025, -0.00416, 100, 0.008, 0.007, 60, 9, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// wait_for_drive_complete();
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	// close_claw();
	// lower_lifter();
	// right_side_rush_backup();
	// raise_lifter();
	// move_arm(475, 100);
	// async_curve_drive_to_point(10_in, 50_in, 100, 20, 0.025, -0.00416, 60, 0.005, 0.006, 80, 10, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// pros::delay(300);
	// async_intake(600, false);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(24_in, 56_in, 100, 20, 0.025, -0.00416, 100, 0.007, 0.006, 60, 9, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// wait_for_drive_complete_2();
	// async_curve_drive_to_point(0_in, 10_in, 100, 20, 0.025, -0.00416, 100, 0.012, 0.007, 100, 10, true, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);
	// pros::delay(750);
	// move_arm(0, 100);
	// wait_for_drive_complete_2();
	// lower_lifter();
	// gyro_drive(2_in, 0.09, 100, 15, true, 0.02, 0.04);//0.04


	////////////////////////////////////////////////////////////////////IDK_WHAT_THIS_DOES --SLOT 5///////////////////////////////////////////////////////////////////////////////

	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::hold);
	// open_claw();
	// move_arm(-50, 100);
	// // stick.set_value(false);
	// lower_flap();
	// async_gyro_drive(34_in, 1.0, 100, 95, true, 0.02, 0.1);//0.04
	// wait_for_drive_complete();
	// drive_train.SetBrakeMode(AbstractMotor::brakeMode::coast);
	// close_claw();
	// move_arm(75, 33);
	// right_side_rush_mid_goal();
	// raise_lifter();
	// raise_flap();
	// async_curve_drive_to_point(-5_in, 17_in, 100, 20, 0.025, -0.00416, 60, 0.005, 0.006, 80, 10, false, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// wait_for_drive_complete_2();
	// tilter.set_value(true);
	// pros::delay(500);
	// back_claw.set_value(false);
	// async_curve_drive_to_point(17.5_in, 19_in, 100, 20, 0.025, -0.00416, 60, 0.005, 0.006, 80, 10, true, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);
	// wait_for_drive_complete_2();
	// raise_lifter();
	// move_arm(500, 100);
	// async_curve_drive_to_point(10_in, 50_in, 100, 30, 0.025, -0.00416, 60, 0.005, 0.006, 80, 10, true, false, false, NONE, 3, 0.00001, 2, 28, 1, 30);
	// pros::delay(300);
	// async_intake(600, false);
	// wait_for_drive_complete_2();
	// // async_curve_drive_to_point(24_in, 56_in, 100, 20, 0.025, -0.00416, 100, 0.007, 0.006, 60, 9, true, false, false, NONE, 3, 0.00001, 2, 28, 1);
	// // wait_for_drive_complete_2();
	// async_curve_drive_to_point(10_in, 15_in, 100, 30, 0.035, -0.00416, 100, 0.012, 0.007, 100, 10, true, false, true, NONE, 3, 0.00001, 2, 28, 1, 30);
	// pros::delay(750);
	// move_arm(0, 100);
	// wait_for_drive_complete_2();
	// lower_lifter();
	// gyro_drive(2_in, 0.09, 100, 15, true, 0.02, 0.04);//0.04
	// // curve_drive_to_point(-3_in, 12_in, 100, 20, 0.025, -0.00416, 100, 0.012, 0.007, 100, 10, false, false, false, NONE, 3, 0.00001, 2, 28, 1);


	///////////////////////////////////////////////////////////////////////FAR_JUKE --SLOT 6///////////////////////////////////////////////////////////////////////

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
