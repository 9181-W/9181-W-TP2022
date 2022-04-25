#include "okapi/api.hpp"
#include "inertial.hpp"
#include "initialize.hpp"
#include "position_tracker.hpp"
#include "drive_train.hpp"
#include "vision.hpp"
#include "vision_drive.hpp"
#include "math.h"
using namespace okapi;

double sqr_3(double value)
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

double current_x_position_3 = get_x_position();
double current_y_position_3 = get_y_position();

double get_adj_heading_3(bool reverse)
{
  double angle = get_heading().convert(degree);

  if(reverse == true)
  {
      angle = angle - 180;
  }

  return angle;
}

double worldspace_target_heading_3(double x, double y)
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

double find_closest_angle_3(double x_target_position, double y_target_position, bool reverse)
{
  double new_angle = 0.0;
  double pos_angle_1 = fabs(worldspace_target_heading_3(x_target_position, y_target_position) - get_adj_heading_3(reverse));
  double pos_angle_2 = fabs((worldspace_target_heading_3(x_target_position, y_target_position) + 360.0) - get_adj_heading_3(reverse));
  double pos_angle_3 = fabs((worldspace_target_heading_3(x_target_position, y_target_position) - 360.0) - get_adj_heading_3(reverse));

  if((pos_angle_1 <= pos_angle_2) && (pos_angle_1 <= pos_angle_3))
  {
    new_angle = worldspace_target_heading_3(x_target_position, y_target_position);
  }
  else if((pos_angle_2 <= pos_angle_1) && (pos_angle_2 <= pos_angle_3))
  {
    new_angle = worldspace_target_heading_3(x_target_position, y_target_position) + 360.0;
  }
  else if((pos_angle_3 <= pos_angle_1) && (pos_angle_3 <= pos_angle_2))
  {
    new_angle = worldspace_target_heading_3(x_target_position, y_target_position) - 360.0;
  }

  return new_angle;
}

void curve_drive_to_line(QLength x_target, QLength y_target, double max_speed, double min_speed, double kp, double kd, double max_turn_speed, double turn_kp, bool reverse, double epsilon)
{
  double x_target_position = x_target.convert(inch);
  double y_target_position = y_target.convert(inch);

  current_x_position_3 = get_x_position();
  current_y_position_3 = get_y_position();

  double x_current_position = get_x_position();
  double y_current_position = get_y_position();

  double A = x_target_position - x_current_position;
  double B = y_target_position - y_current_position;
  double C = -1 * (((x_target_position - x_current_position) * x_target_position) + ((y_target_position - y_current_position) * y_target_position));

  double new_angle = find_closest_angle_3(x_target_position, y_target_position, reverse);

  //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
  max_speed = max_speed / 100;
  min_speed = min_speed / 100;
  max_turn_speed = max_turn_speed / 100;

  //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
  //and have no jerk at the beggining
  const double maximum_vel_adj = 0.1;

  //Converts Qlength distance to distance_in_inches
  double initial_distance_to_target = (fabs((A * x_current_position) + (B * y_current_position) + C)) / (sqrt(sqr_3(A) + sqr_3(B)));
  //Defines the initial drive error (found in chassisController.cpp on github)
  double drive_error = (fabs((A * x_current_position) + (B * y_current_position) + C)) / (sqrt(sqr_3(A) + sqr_3(B)));
  //sets the initial worldspace angle between the current position in worldspace to the target position in worldspace
  double angle_to_target = worldspace_target_heading_3(x_target_position, y_target_position);

  //Sets last error to zero before driving starts
  double last_error = 0.0;

  double second_last_error = 9999.9;
  double third_last_error = 9999.9;
  double fourth_last_error = 9999.9;
  //Creates a variable that contains the initial gyro value (0)
  // inertial_reset();
  double initial_drive_gyro_value = inertial_get_value();

  //Sets the first speed to zero
  double last_speed = drive_train.GetVelocity();
  if(reverse == true)
  {
    last_speed = last_speed * -1;
  }

  double last_three_derivatives = 9999.9;

  double smallest_error = 9999.0;

  while(drive_error > epsilon)
  {
    current_x_position_3 = get_x_position();
    current_y_position_3 = get_y_position();

    drive_error = (fabs((A * current_x_position_3) + (B * current_y_position_3) + C)) / (sqrt(sqr_3(A) + sqr_3(B)));

    //Calculates the derivative
    double derivative = last_error - drive_error;

    //Sets a new last error
    last_error = drive_error;

    //Calculate speed to be driven at using kp,ki,kd
    // 41.6122 degrees or rotation per inch.
    double speed = drive_error * kp  + derivative * kd;

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

    double velocity_adj = speed - last_speed;

    if(velocity_adj > maximum_vel_adj)
    {
        speed = last_speed + maximum_vel_adj;
        // speed = speed;
    }

     if(velocity_adj < -maximum_vel_adj)
    {
        speed = last_speed + -maximum_vel_adj;
        // speed = speed;
    }

    last_speed = speed;

    if(reverse == true)
    {
      speed = speed * -1;
    }

    // ****************************************************************************************************************************** //
    // This code will make the robot drive straight by turning small distances if the robot has driven slightly to the right or left  //
    // - This does not support driving backwards right now.                                                                           //
    // ****************************************************************************************************************************** //
    double turn_speed = 0.0;

    //Calculates the amount that the robot is off of its heading
    double curve_error = new_angle - get_adj_heading_3(reverse);

    //Creates a turn speed so that different sides can be slowed down
    turn_speed = curve_error * turn_kp;
    if(turn_speed > max_turn_speed)
    {
      turn_speed = max_turn_speed;
    }
    else if(turn_speed < -max_turn_speed)
    {
      turn_speed = -max_turn_speed;
    }

    printf("drive_error: %7.3f  heading: %7.3f  angle_to_target: %7.3f  curve_error: %7.3f  turn_speed: %7.3f speed: %7.3f\n",drive_error,inertial_get_value(),new_angle,curve_error,turn_speed,speed);

    //Setting the desired speed in a percent form and waiting 10 milliseconds
    drive_train.AutonomousArcadeDrive(speed, -turn_speed, false);
    pros::delay(33);
  }
  //Stops the robot from moving after the robot has reached its target distance
  drive_train.AutonomousArcadeDrive(0.0, 0.0, true);
}
