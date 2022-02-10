#include "okapi/api.hpp"
#include "inertial.hpp"
#include "initialize.hpp"
#include "position_tracker.hpp"
#include "math.h"
#include "drive_train.hpp"
#include "initialize.hpp"
using namespace okapi;

double sqr_2(double value)
{
  return value*value;
}

//Creates a constant for wheel diameter
const double wheel_diam = 2.75;
//Creates a constant for pi
const double Pi = 3.14159265359;
//Calculates a constant wheel circumference using diameter and pi
const double wheel_circ = wheel_diam * Pi;
//Encoder degrees in circumference
const double degrees_per_circ = 360.0;
//Encoder degrees per inch
const double degrees_per_inch = degrees_per_circ / wheel_circ;

const double zero_speed = 0.03;

double current_x_position_2 = get_x_position();
double current_y_position_2 = get_y_position();

double get_adj_heading_2(bool reverse)
{
  double angle = get_heading().convert(degree);

  if(reverse == true)
  {
    angle = angle + 180;
  }

  return angle;
}

double worldspace_target_heading_2(double x, double y)
{
  double curr_x = get_x_position();
  double curr_y = get_y_position();

  double x_diff = x - curr_x;
  double y_diff = y - curr_y;

  double target_heading = 0.0;

  if((y_diff == 0.0) && (x_diff >= 0.0))
  {
    target_heading = 90.0;
  }
  else if((y_diff == 0.0) && (x_diff <= 0.0))
  {
    target_heading = -90.0;
  }
  else
  {
    target_heading = (atan(x_diff / y_diff)) * (180 / Pi);

    if((y_diff < 0.0) && (x_diff < 0.0))
    {
      target_heading = -180.0 + target_heading;
    }
    else if((y_diff < 0.0) && (x_diff >= 0.0))
    {
      target_heading = 180.0 + target_heading;
    }
  }

  return target_heading;
}

double find_closest_angle_2(double x_target_position, double y_target_position, bool reverse)
{
  double new_angle = 0.0;
  double pos_angle_1 = fabs(worldspace_target_heading_2(x_target_position, y_target_position) - get_adj_heading_2(reverse));
  double pos_angle_2 = fabs((worldspace_target_heading_2(x_target_position, y_target_position) + 360.0) - get_adj_heading_2(reverse));
  double pos_angle_3 = fabs((worldspace_target_heading_2(x_target_position, y_target_position) - 360.0) - get_adj_heading_2(reverse));

  if((pos_angle_1 <= pos_angle_2) && (pos_angle_1 <= pos_angle_3))
  {
    new_angle = worldspace_target_heading_2(x_target_position, y_target_position);
  }
  else if((pos_angle_2 <= pos_angle_1) && (pos_angle_2 <= pos_angle_3))
  {
    new_angle = worldspace_target_heading_2(x_target_position, y_target_position) + 360.0;
  }
  else if((pos_angle_3 <= pos_angle_1) && (pos_angle_3 <= pos_angle_2))
  {
    new_angle = worldspace_target_heading_2(x_target_position, y_target_position) - 360.0;
  }

  return new_angle;
}

// double get_robot_velocity_2()
// {
//   // double robot_velocity = left_rear_mtr.getActualVelocity() + left_front_mtr.getActualVelocity() + right_rear_mtr.getActualVelocity() + right_front_mtr.getActualVelocity();
//   // robot_velocity = robot_velocity / 4.0; //averaging
//   // robot_velocity = robot_velocity / 200.0; //converting from RPM to Analog
//   double robot velocity = drive_train.GetVelocity();
//
//   return robot_velocity;
// }

void drive_through_point(QLength x_target, QLength y_target, double max_speed, double min_speed, double kp, double kd, double max_turn_speed, double turn_kp, bool reverse, double rotate_epsilon, double epsilon, double division_const)
{
  double x_target_position = x_target.convert(inch);
  double y_target_position = y_target.convert(inch);

  double new_angle = 0.0;

  //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
  max_speed = max_speed / 100;
  min_speed = min_speed / 100;
  max_turn_speed = max_turn_speed / 100;

  //Creates a constant for allowable error before stopping
  epsilon = 0.5;
  //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
  //and have no jerk at the beggining
  const double maximum_vel_adj = 0.1;
  //Sets the proportional constant for driving straight
  //const double drive_straight_kp = 0.05;
  // const double turn_kp = 0.05;

  //Converts Qlength distance to distance_in_inches
  double initial_distance_to_target = sqrt(sqr_2(x_target_position - current_x_position_2) + sqr_2(y_target_position - current_y_position_2));
  //Defines the initial drive error (found in chassisController.cpp on github)
  double drive_error = sqrt(sqr_2(x_target_position - current_x_position_2) + sqr_2(y_target_position - current_y_position_2));
  //sets the initial worldspace angle between the current position in worldspace to the target position in worldspace
  double angle_to_target = worldspace_target_heading_2(x_target_position, y_target_position);

  //Sets last error to zero before driving starts
  double last_error = 0.0;

  double second_last_error = 9999.9;
  double third_last_error = 9999.9;
  double fourth_last_error = 9999.9;
  //Creates a variable that contains the initial gyro value (0)
  // inertial_reset();
  double initial_drive_gyro_value = inertial_get_value();

  //Sets the first speed to zero
  double last_speed = 0.0;

  double last_three_derivatives = 9999.9;
  // ****************************** //
  //  Rotate While Loop Variables   //
  // ****************************** //

  double rotate_distance = worldspace_target_heading_2(x_target_position, y_target_position) - get_adj_heading_2(reverse);
  double rotate_error = rotate_distance;

  while(drive_error > 5)
  {
    // pros::lcd::print(1,"actual_robot_velocity= %5.1f", get_robot_velocity_2());

    current_x_position_2 = get_x_position();
    current_y_position_2 = get_y_position();

    drive_error = sqrt(sqr_2(x_target_position - current_x_position_2) + sqr_2(y_target_position - current_y_position_2));

    //Calculates the derivative
    double derivative = last_error - drive_error;

    fourth_last_error = third_last_error;

    third_last_error = second_last_error;

    second_last_error = last_error;

    //Sets a new last error
    last_error = drive_error;

    double third_last_derivative = fourth_last_error - third_last_error;

    double second_last_derivative = third_last_error - second_last_error;

    double last_derivative = second_last_error - last_error;

    last_three_derivatives = last_derivative + second_last_derivative + third_last_derivative;

    //Calculate speed to be driven at using kp,ki,kd
    double speed = drive_error * kp  + derivative * kd;
    // + integral * drive_ki
    //printf("Speed: %f  (p,i,d): (%f,%f,%f) ",speed,drive_error*drive_kp,integral*drive_ki,derivative*drive_kd);
    printf("req speed: %f\n",speed);

    //Removes impossible speeds by setting the speed down to a possible one
    if(speed > max_speed)
    {
        speed = max_speed;
    }
    if(speed < max_speed * -1)
    {
        speed = max_speed * -1;
    }
    //If the speed is lower than a certain value set the speeed to zero otherwise if the robot is slower than the minimum speed set the robot to the minimum speed
    // printf("speed %f zero_speed %f\n",fabs(speed),zero_speed);
    if(fabs(speed) < zero_speed)
    {
      speed = 0.0;
    }
    else if(fabs(speed) < min_speed)
    {
        if(speed > 0)
        {
            speed = min_speed;
        }

        else if(speed < 0)
        {
            speed = min_speed * -1;
        }
    }

    // ************************************************************************************************* //
    // Set maximum accelleration to prevent the robot from jerking right or left at the start of driving //
    // ************************************************************************************************* //

    //dont allow the robot to slow down
    if(fabs(last_speed) > fabs(speed))
    {
      speed = last_speed;
    }

    double velocity_adj = speed - last_speed;

    if(velocity_adj > maximum_vel_adj)
    {
        speed = last_speed + maximum_vel_adj;
    }

     if(velocity_adj < -maximum_vel_adj)
    {
        speed = last_speed + -maximum_vel_adj;
    }

    last_speed = speed;

    if(reverse == true)
    {
      speed = speed * -1;
    }

    // printf("last_three_derivatives: %f\n",fabs(last_three_derivatives));
    // printf("adj speed: %f\n",speed);
    // printf("drive error: %f\n",drive_error);

    // ****************************************************************************************************************************** //
    // This code will make the robot drive straight by turning small distances if the robot has driven slightly to the right or left  //
    // - This does not support driving backwards right now.                                                                           //
    // ****************************************************************************************************************************** //
    double turn_speed = 0.0;
    new_angle = find_closest_angle_2(x_target_position, y_target_position, reverse);
    //when less than 20% of the distance is less then continue driving straight
    if(((drive_error / initial_distance_to_target) * 100) > 20)
    {
      angle_to_target = new_angle;
    }

    //Calculates the amount that the robot is off of its heading
    double curve_error = angle_to_target - get_adj_heading_2(reverse);
    // printf("curve_error: %f\n",curve_error);
    printf("last_three_derivs: %f\n", last_three_derivatives);


    //Creates a turn speed so that different sides can be slowed down
    turn_speed = curve_error * turn_kp;
    // printf("req_turn_speed: %f\n",turn_speed);

    if(turn_speed > max_turn_speed)
    {
      turn_speed = max_turn_speed;
    }
    else if(turn_speed < -max_turn_speed)
    {
      turn_speed = -max_turn_speed;
    }

    // ******************************************* //
    // Set final speed and calculate the new error //
    // ******************************************* //

    pros::lcd::print(2,"requested_velocity= %5.1f", speed);

    //Setting the desired speed in a percent form and waiting 10 milliseconds
    // std::shared_ptr<ChassisModel> chassis_model = chassis->getModel();
    // chassis_model->arcade(speed, turn_speed);
    drive_train.AutonomousArcadeDrive(speed, -turn_speed);
    // chassis_model->driveVector(speed, (turn_speed / 4));
    pros::delay(33);
  }

  pros::lcd::print(7,"EXITED");
  printf("exited\n");

  //Stops the robot from moving after the robot has reached its target distance
  drive_train.AutonomousArcadeDrive(0.0, 0.0);
}
