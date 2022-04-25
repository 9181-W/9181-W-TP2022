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

void drive_through_point(QLength x_target, QLength y_target, double max_speed, double max_turn_speed, double turn_kp, bool reverse, double epsilon, double lookahead_x, double lookahead_y, double lookahead_percentage)
{
  double x_target_position = x_target.convert(inch);
  double y_target_position = y_target.convert(inch);

  double x_current_position = get_x_position();
  double y_current_position = get_y_position();

  double A = x_target_position - x_current_position;
  double B = y_target_position - y_current_position;
  double C = -1 * (((x_target_position - x_current_position) * x_target_position) + ((y_target_position - y_current_position) * y_target_position));

  double new_angle = 0.0;

  //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
  max_speed = max_speed / 100;
  max_turn_speed = max_turn_speed / 100;

  //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
  //and have no jerk at the beggining
  const double maximum_vel_adj = 0.1;

  double initial_distance_to_target = (fabs((A * x_current_position) + (B * y_current_position) + C)) / (sqrt(sqr_2(A) + sqr_2(B)));
  //Defines the initial drive error (found in chassisController.cpp on github)
  double drive_error = (fabs((A * x_current_position) + (B * y_current_position) + C)) / (sqrt(sqr_2(A) + sqr_2(B)));
  //sets the initial worldspace angle between the current position in worldspace to the target position in worldspace
  //double angle_to_target = worldspace_target_heading_2(x_target_position, y_target_position);
  double angle_to_target = find_closest_angle_2(x_target_position, y_target_position, reverse);
  //double desired_angle = worldspace_target_heading_2(x_target_position, y_target_position);
  double desired_angle = find_closest_angle_2(x_target_position, y_target_position, reverse);

  double initial_drive_gyro_value = inertial_get_value();

  //Sets the first speed to zero
  double last_speed = drive_train.GetVelocity();
  if(reverse == true)
  {
    last_speed = last_speed * -1;
  }

  double turn_speed = 0.0;
  double curve_error = 0.0;

  printf("Target (%5.1f, %5.1f)  Heading %5.1f\n",x_target_position,y_target_position,angle_to_target);

  while(drive_error > (epsilon + sqr_2(max_speed)))
  {
    current_x_position_2 = get_x_position();
    current_y_position_2 = get_y_position();

    drive_error = (fabs((A * current_x_position_2) + (B * current_y_position_2) + C)) / (sqrt(sqr_2(A) + sqr_2(B)));

    double speed = max_speed;

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

    // ****************************************************************************************************************************** //
    // This code will make the robot drive straight by turning small distances if the robot has driven slightly to the right or left  //
    // - This does not support driving backwards right now.                                                                           //
    // ****************************************************************************************************************************** //
    turn_speed = 0.0;

    //when less than 20% of the distance is less then continue driving straight
    if(((drive_error / initial_distance_to_target) * 100) > lookahead_percentage)
    {
      // angle_to_target = find_closest_angle_2(x_target_position, y_target_position, reverse);
      angle_to_target = desired_angle;
    }
    else
    {
      angle_to_target = find_closest_angle_2(lookahead_x, lookahead_y, reverse);
    }

    //Calculates the amount that the robot is off of its heading
    curve_error = angle_to_target - get_adj_heading_2(reverse);

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

    // static int count = 0;
    // if ((count % 15) == 0)
    // {
    //   printf("Current (%5.1f, %5.1f)  Error %5.1f %5.1f  DSpeed %7.3f  TSpeed %5.1f  Heading %5.1f\n",current_x_position_2,current_y_position_2,drive_error,curve_error,speed,turn_speed,get_adj_heading_2(reverse));
    // }

    drive_train.AutonomousArcadeDrive(speed, -turn_speed, false);

    pros::delay(10);
  }

  printf("Current (%5.1f, %5.1f)  Error %5.1f %5.1f  TSpeed %5.1f  Heading %5.1f\n",current_x_position_2,current_y_position_2,drive_error,curve_error,turn_speed,get_adj_heading_2(reverse));
  printf("Target: (%5.1f, %5.1f)  Exited (%5.1f %5.1f)\n",x_target_position,y_target_position,get_x_position(),get_y_position());
}
