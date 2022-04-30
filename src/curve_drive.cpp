#include "okapi/api.hpp"
#include "inertial.hpp"
#include "initialize.hpp"
#include "position_tracker.hpp"
#include "drive_train.hpp"
#include "vision.hpp"
#include "vision_drive.hpp"
#include "math.h"
using namespace okapi;

double sqr(double value)
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

double current_x_position = get_x_position();
double current_y_position = get_y_position();

double get_adj_heading(bool reverse)
{
  double angle = get_heading().convert(degree);

  if(reverse == true)
  {
    // if(angle > 360)
    // {
      angle = angle - 180;
    // }
    // else
    // {
    //   angle = angle + 180;
    // }
  }

  return angle;
}

double worldspace_target_heading(double x, double y)
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

double find_closest_angle(double x_target_position, double y_target_position, bool reverse)
{
  double new_angle = 0.0;
  double pos_angle_1 = fabs(worldspace_target_heading(x_target_position, y_target_position) - get_adj_heading(reverse));
  double pos_angle_2 = fabs((worldspace_target_heading(x_target_position, y_target_position) + 360.0) - get_adj_heading(reverse));
  double pos_angle_3 = fabs((worldspace_target_heading(x_target_position, y_target_position) - 360.0) - get_adj_heading(reverse));

  if((pos_angle_1 <= pos_angle_2) && (pos_angle_1 <= pos_angle_3))
  {
    new_angle = worldspace_target_heading(x_target_position, y_target_position);
  }
  else if((pos_angle_2 <= pos_angle_1) && (pos_angle_2 <= pos_angle_3))
  {
    new_angle = worldspace_target_heading(x_target_position, y_target_position) + 360.0;
  }
  else if((pos_angle_3 <= pos_angle_1) && (pos_angle_3 <= pos_angle_2))
  {
    new_angle = worldspace_target_heading(x_target_position, y_target_position) - 360.0;
  }

  return new_angle;
}

void curve_drive_to_point(QLength x_target, QLength y_target, double max_speed, double min_speed, double kp, double kd, double max_turn_speed, double turn_kp, double rotate_kp, double max_rotate_speed, double min_rotate_speed, bool turn_first, bool stop_drive, bool reverse, int colour, double rotate_epsilon, double epsilon, double division_const, double lookahead_distance, double true_epsilon, double straight_dist)
{
  double x_target_position = x_target.convert(inch);
  double y_target_position = y_target.convert(inch);
  // pros::lcd::print(5,"ENTERED");

  current_x_position = get_x_position();
  current_y_position = get_y_position();

  double new_angle = 0.0;

  //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
  max_speed = max_speed / 100;
  min_speed = min_speed / 100;
  max_rotate_speed = max_rotate_speed / 100;
  min_rotate_speed = min_rotate_speed / 100;
  max_turn_speed = max_turn_speed / 100;

  //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
  //and have no jerk at the beggining
  const double maximum_vel_adj = 0.1;
  //Sets the proportional constant for driving straight
  //const double drive_straight_kp = 0.05;
  // const double turn_kp = 0.05;

  //Converts Qlength distance to distance_in_inches
  double initial_distance_to_target = sqrt(sqr(x_target_position - current_x_position) + sqr(y_target_position - current_y_position));
  //Defines the initial drive error (found in chassisController.cpp on github)
  double drive_error = sqrt(sqr(x_target_position - current_x_position) + sqr(y_target_position - current_y_position));
  //sets the initial worldspace angle between the current position in worldspace to the target position in worldspace
  double angle_to_target = worldspace_target_heading(x_target_position, y_target_position);

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

  double rotate_distance = worldspace_target_heading(x_target_position, y_target_position) - get_adj_heading(reverse);
  double rotate_error = rotate_distance;

  // double max_rotate_speed = 100;
  // double min_rotate_speed = 0;

  int turn_time = 0;

  // while ((fabs(rotate_error) > rotate_epsilon))
  while((fabs(rotate_error) > rotate_epsilon) && (turn_first == true) && (turn_time < 2500))
  {
      // ******************************************************************************************************************************* //
      //  This code uses proportional , differential, and integral constants to calculate the best speed to reach the desired distance   //
      // ******************************************************************************************************************************* //

      // This code accounts for the fact that when the robot turns more than 360 degrees in one direction the gyroscope will still read proper values
      // if(((current_pos_in_degrees + 180) < last_current_pos_in_degrees) && (distance_in_degrees > last_current_pos_in_degrees))
      // {
      //   distance_in_degrees = distance_in_degrees - 360;
      // }
      //
      // else if(((current_pos_in_degrees -180)> last_current_pos_in_degrees) && (distance_in_degrees < last_current_pos_in_degrees))
      // {
      //   distance_in_degrees = distance_in_degrees + 360;
      // }
      //
      // last_current_pos_in_degrees = current_pos_in_degrees;

      new_angle = find_closest_angle(x_target_position, y_target_position, reverse);

      //Calculate distance left to turn
      rotate_error = new_angle - get_adj_heading(reverse);

      //Calculate speed to be turned at using kp
      double rotate_speed = rotate_error * rotate_kp;

      //Removes impossible speeds by setting the speed down to a possible one
      if(rotate_speed > max_rotate_speed)
      {
          rotate_speed = max_rotate_speed;
      }

      if(rotate_speed < max_rotate_speed * -1)
      {
          rotate_speed = max_rotate_speed * -1;
      }

      if ((rotate_speed > 0) && (rotate_speed < min_rotate_speed))
      {
        rotate_speed = min_rotate_speed;
      }
      if ((rotate_speed < 0) && (rotate_speed > -min_rotate_speed))
      {
        rotate_speed = -min_rotate_speed;
      }

      //Setting the desired speed in a percent form and waiting 10 milliseconds
      drive_train.AutonomousArcadeDrive(0.0, -rotate_speed, true);

      // pros::lcd::print(1,"curr_x= %5.1f, curr_y= %5.1f",current_x_position, current_y_position);
      // pros::lcd::print(2,"targ_x= %5.1f, targ_y= %5.1f",x_target_position, y_target_position);
      // pros::lcd::print(3,"error= %5.1f", drive_error);
      // pros::lcd::print(4,"angle= %5.1f", new_angle);
      // pros::lcd::print(5,"inertial_angle= %5.1f", inertial_get_value());

      pros::delay(33);
      turn_time += 33;
  }

  // //Stops the robot from moving after the robot has reached its target distance
  // chassis->getModel()->stop();

  if(stop_drive == true)
  {
    // while(true)
    // {
    //   pros::lcd::print(1,"curr_x= %5.1f, curr_y= %5.1f",current_x_position, current_y_position);
    //   pros::lcd::print(2,"targ_x= %5.1f, targ_y= %5.1f",x_target_position, y_target_position);
    //   pros::lcd::print(3,"rotate error= %5.1f", rotate_error);
    //   pros::lcd::print(4,"angle= %5.1f", new_angle);
    //   pros::lcd::print(5,"inertial_angle= %5.1f", inertial_get_value());
    //
    //   pros::delay(33);
    // }
    return;
  }

  double smallest_error = 9999.0;

  // Emergency EXIT
  // if (drive_error > smallest_error * 1.2)
  // {
  //   printf("EMERGENCY_EXIT");
  //   pros::lcd::print(4,"EMERGENCY EXIT");
  //   break;
  // }

  while((((fabs(drive_error) > true_epsilon) && (fabs(last_three_derivatives)) > epsilon) && (drive_error < smallest_error * 1.2)) || (fabs(drive_error) > fabs(initial_distance_to_target) / division_const))
  // while(((fabs(drive_error) > true_epsilon) && (fabs(last_three_derivatives)) > epsilon) || (fabs(drive_error) > fabs(initial_distance_to_target) / division_const))
  {
    // pros::lcd::print(1,"actual_robot_velocity= %5.1f", drive_train.GetVelocity());
    // pros::lcd::print(3,"last three derivative = %5.1f", last_three_derivatives);
    // pros::lcd::print(4,"drive error           = %5.1f", drive_error);

    current_x_position = get_x_position();
    current_y_position = get_y_position();

    drive_error = sqrt(sqr(x_target_position - current_x_position) + sqr(y_target_position - current_y_position));

    if (drive_error < smallest_error)
    {
      smallest_error = drive_error;
    }

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
    // 41.6122 degrees or rotation per inch.
    double speed = drive_error * kp  + derivative * kd;
    // + integral * drive_ki
    //printf("Speed: %f  (p,i,d): (%f,%f,%f) ",speed,drive_error*drive_kp,integral*drive_ki,derivative*drive_kd);
    //printf("req speed: %f\n",speed);

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

    // printf("last_three_derivatives: %f\n",fabs(last_three_derivatives));
    // printf("adj speed: %f\n",speed);
    // printf("drive error: %f\n",drive_error);

    // ****************************************************************************************************************************** //
    // This code will make the robot drive straight by turning small distances if the robot has driven slightly to the right or left  //
    // - This does not support driving backwards right now.                                                                           //
    // ****************************************************************************************************************************** //
    double turn_speed = 0.0;
    new_angle = find_closest_angle(x_target_position, y_target_position, reverse);

    //Calculates the amount that the robot is off of its heading
    double curve_error = new_angle - get_adj_heading(reverse);
      //printf("last_three_derivs: %f\n", last_three_derivatives);


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

    // when less than 20% of the distance is less then continue driving straight
    if(((drive_error / initial_distance_to_target) * 100.0) < straight_dist)
    {
      turn_speed = 0;
    }

    // printf("driver_error: %7.3f  heading: %7.3f  angle_to_target: %7.3f  curve_error: %7.3f  turn_speed: %7.3f\n",drive_error,inertial_get_value(),new_angle,curve_error,turn_speed);
    // pros::lcd::print(3,"curve_error= %5.3f", curve_error);
    // pros::lcd::print(4,"turn_speed= %5.3f", turn_speed);
    // pros::lcd::print(7,"angle_to_target= %5.3f", angle_to_target);
    // printf("adj_turn_speed: %f\n",turn_speed);



    // pros::lcd::print(1,"curr_x= %5.1f, curr_y= %5.1f",current_x_position, current_y_position);
    // pros::lcd::print(2,"targ_x= %5.1f, targ_y= %5.1f",x_target_position, y_target_position);
    // pros::lcd::print(3,"error= %5.1f", drive_error);
    // pros::lcd::print(4,"angle= %5.1f", angle_to_target);
    // pros::lcd::print(5,"inertial_angle= %5.1f", inertial_get_value());



    // ******************************************* //
    // Set final speed and calculate the new error //
    // ******************************************* //

    // pros::lcd::print(2,"requested_velocity= %5.1f", speed);
    // pros::lcd::print(7,"smallest_error= %7.3",smallest_error);

    // Emergency EXIT
    // if (drive_error > smallest_error * 1.2)
    // {
    //   printf("EMERGENCY_EXIT");
    //   pros::lcd::print(4,"EMERGENCY EXIT");
    //   break;
    // }

    double current_center = 0.0;
    double current_area = 0.0;

    if ((drive_error < lookahead_distance) && (colour != NONE))
    {
      if(colour == RED_GOAL)
      {
        current_center = red_center;
        current_area = red_area;
      }

      if(colour == BLUE_GOAL)
      {
        current_center = blue_center;
        current_area = blue_area;
      }

      if(colour == YELLOW_GOAL)
      {
        current_center = yellow_center;
        current_area = yellow_area;
      }

      // printf("current_area %f\n", current_area);

      if(current_area > 25.0)
      {
        return;
      }
    }


    //Setting the desired speed in a percent form and waiting 10 milliseconds
    drive_train.AutonomousArcadeDrive(speed, -turn_speed, true);
    // drive_train.ArcadeDrive(speed, -turn_speed);
    // chassis_model->driveVector(speed, (turn_speed / 4));
    // pros::lcd::print(4,"ENTERED");
    pros::lcd::clear_line(5);
    pros::delay(33);
    // while(((fabs(drive_error) > 1.0) && (fabs(last_three_derivatives)) > epsilon) || (fabs(drive_error) > fabs(initial_distance_to_target) / division_const))
    // printf("SPEED= %7.3f, ZERO_SPEED= %7.3f\n", speed, zero_speed);
    // if ((fabs(last_three_derivatives)) < epsilon)
    // {
    //   printf("exit_by_speed  ERROR= %7.3f, LTD= %7.3f\n", drive_error, last_three_derivatives);
    // }
    // printf("e1 %7.4f, e2 %7.4f, e3 %7.4f, e4 %7.4f\n", last_error, second_last_error, third_last_error, fourth_last_error);
    // printf("d1 %7.4f, d2 %7.4f, d3 %7.4f, l3d %7.4f\n", last_derivative, second_last_derivative, third_last_derivative, last_three_derivatives);
    // printf("fabs(l3d): %7.3f > %7.3f\n", fabs(last_three_derivatives), epsilon);
    // printf("div const: %7.3f > %7.3f\n", fabs(drive_error), (fabs(initial_distance_to_target) / division_const));
  }

  // pros::lcd::print(6,"EXITED");

  // pros::lcd::print(5,"EXIT: %5.1f - %5.1f - %5.1f",drive_error,smallest_error,last_three_derivatives);
  // pros::lcd::print(6,"X: %5.1f  Y: %5.1f",current_x_position,current_y_position);
  // printf("exited\n");

  //Stops the robot from moving after the robot has reached its target distance
  // ((fabs(drive_error) > true_epsilon) && (fabs(last_three_derivatives)) > epsilon) || (fabs(drive_error) > fabs(initial_distance_to_target) / division_const))

  printf("DRIVE_ERROR= %7.3f, TRUE_EPSILON= %7.3f, EPSILON= %7.3f\n", drive_error, true_epsilon, epsilon);
  printf("LTD= %7.3f, IDTT= %7.3f, DIV_CONST= %7.3f\n", last_three_derivatives, initial_distance_to_target, division_const);
  printf("CURRENT_X= %7.3f, CURRENT_Y= %7.3f\n", current_x_position, current_y_position);
  printf("TARGET_X= %7.3f, TARGET_Y= %7.3f\n", x_target_position, y_target_position);
  printf("EXITED\n");

  drive_train.AutonomousArcadeDrive(0.0, 0.0, true);
}
