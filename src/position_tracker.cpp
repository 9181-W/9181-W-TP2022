#include "okapi/api.hpp"
#include "inertial.hpp"
#include "initialize.hpp"

using namespace okapi;

// std::shared_ptr<ChassisController> chassis =
//   ChassisControllerBuilder()
//     .withMotors(
//       // LEFT_FRONT_WHEEL_PORT, -RIGHT_FRONT_WHEEL_PORT,
//       // RIGHT_REAR_WHEEL_PORT, -LEFT_REAR_WHEEL_PORT
//       {-LEFT_FRONT_WHEEL_PORT, -LEFT_REAR_WHEEL_PORT},
//       {RIGHT_FRONT_WHEEL_PORT, RIGHT_REAR_WHEEL_PORT}
//     )
//     .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 14_in}, imev5GreenTPR})//4, 14
//     .build();

//Link to 5225A github github.com/nickmertin/5225A-2017-2018/blob/master/src/auto.c
//
// The following algorithm came from: http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
//
// The following variables are used in this document to represent physical parameters and other known
// values:
// β’ π πΏ   is the left-right distance from the tracking center to the left tracking wheel
// β’ π π   is the left-right distance from the tracking center to the right tracking wheel
// β’ π π   is the forward-backward distance from the tracking center to the back tracking wheel
// β’ π0  ββββ is the previous global position vector
// β’ π0   is the previous global orientation
// β’ ππ   is the global orientation at last reset
//
// Tracking Algorithm
// The algorithm itself consists of a single procedure to perform calculations, which should be called
// frequently (a cycle period of no more than 10 milliseconds). We recommend using a dedicated runtime
// task/thread for this process.
//
// The procedure can be broken down into a few steps:
//
// 1. Store the current encoder values in local variables
//
// 2. Calculate the change in each encodersβ value since the last cycle, and convert to distance of
// wheel travel (for example, if the wheel has a radius 2" and turned 5Β°, then it travelled
// approximately 0.1745"); call these ΞπΏ, Ξπ, and Ξπ
//
// 3. Update stored "previous values" of encoders
//
// 4. Calculate the total change in the left and right encoder values since the last reset, and convert
// to distance of wheel travel call these ΞπΏπ and Ξππ
//
// 5. Calculate new absolute orientation π1 = ππ + (ΞπΏπβΞππ / π πΏ+π π)
// ; please note that the second term will be in radians, regardless of the units of other variables

// Looking at 5225A's code you can see that step 4 + 5 are ignored and Ξπ is calculated as below

//
// 6. Calculate the change in angle Ξπ = (ΞπΏ - Ξπ / sL + sR)
//
// 7. If Ξπ = 0 (i.e. ΞπΏ = Ξπ), then calculate the local offset Ξππβ = [ Ξπ  Ξπ ]
//
// 8. Otherwise, calculate the local offset Ξπββ = 2 sin π / 2 x [Ξπ / Ξπ + π π   Ξπ / Ξπ + π π]
//
// 9. Calculate the average orientation ππ = π0 + Ξπ / 2
//
// 10. Calculate global offset Ξπβ  as Ξπββ rotated by βππ; this can be done by converting your existing
// Cartesian coordinates to polar coordinates, changing the angle, then converting back
//
// 11. Calculate new absolute position π1βββ = π0 + Ξπβ

//Globals
double x_position = 0.0;
double y_position = 0.0;
QAngle heading = 0.0_deg;

//runs position tracking code
void position_tracker_task(void* param)
{
  //The left-right distance from the tracking center to the left tracking wheel
  //const double sL = 6.5;8.375
  // const double sL = 4.1875;+
  //The left-right distance from the tracking center to the right tracking wheel
  //const double sR = 6.5;
  const double sR = 0.25;
  //The forward-backward distance from the tracking center to the back tracking wheel
  // const double sS = 3.0;
  const double sS = 2.1875;
  // const double sS = 2.05;
  //Creates a constant for wheel diameter
  const double wheel_diam = 2.75;
  //const double wheel_diam = 3.25;
  //Creates a constant for pi
  const double drive_pi = 3.14159265359;
  //Calculates a constant wheel circumference using diameter and pi
  const double wheel_circ = wheel_diam * drive_pi;
  //Encoder degrees in circumference
  const double degrees_per_circ = 360.0;
  //Encoder degrees per inch
  const double degrees_per_inch = degrees_per_circ / wheel_circ;

  //Creates variables for storing the previous linear value of the shaft encoders
  double previous_left_shaft_val = 0.0;
  double previous_right_shaft_val = 0.0;
  double previous_middle_shaft_val = 0.0;
  double previous_inertial_value = 0.0;

  //Creates a variable to keep track of the last calculated angle
  QAngle theta_0 = 0_deg;

  while(true)
  {
    // 1. Store the current encoder values in local variables
    // double left_shaft_val = (shaft_enc_l->get() / degrees_per_inch);
    double right_shaft_val = ((shaft_enc_r->get_position() / 100.0) / degrees_per_inch);
    double middle_shaft_val = -((shaft_enc_m->get_position() / 100.0) / degrees_per_inch);
    double inertial_value = inertial_get_value();

    //2. Calculate the change in each encodersβ value since the last cycle
    // double delta_L = (left_shaft_val - previous_left_shaft_val);
    double delta_R = (right_shaft_val - previous_right_shaft_val);
    double delta_S = (middle_shaft_val - previous_middle_shaft_val);
    double delta_heading = (inertial_value - previous_inertial_value);

    pros::lcd::print(0,"r %5.1f m %5.1f", right_shaft_val, middle_shaft_val);
    // printf("r %5.1f m %5.1f\n", right_shaft_val, middle_shaft_val);

    //3. Update stored "previous values" of encoders
    // previous_left_shaft_val = left_shaft_val;
    previous_right_shaft_val = right_shaft_val;
    previous_middle_shaft_val = middle_shaft_val;
    previous_inertial_value = inertial_value;

    //6. Calculate the change in angle Ξπ = (ΞπΏ - Ξπ / sL + sR)
    //QAngle delta_theta = (((delta_L - delta_R) / (sL + sR)) * radian);
    QAngle delta_theta = delta_heading * degree;

    //7. If Ξπ = 0 (i.e. ΞπΏ = Ξπ), then calculate the local offset Ξππβ = [ Ξπ  Ξπ ]
    // We need an If statement for driving straight because in step 8 we calculate sinΞπ (Ξπ = 0 when driving straight) and the sin of 0 is = infinity (this will screw up the code)

    // creates a variable to store the change in
    double delta_d_x = 0.0;
    double delta_d_y = 0.0;

    //if (delta_theta == 0.0_deg)// cannot do an equal sign here because floats cannot be compared to zero because it will never be exactly 0 (merci beaucoup mon pΓ¨re)
    if ((delta_theta > -0.0001_deg) && (delta_theta < 0.0001_deg))
    {
      delta_d_x = delta_S;
      delta_d_y = delta_R; // Can be R or L because when driving straight R and L will be the Same
    }

    //8. Otherwise, calculate the local offset Ξπββ = 2 sin π / 2 x [Ξπ / Ξπ + π π   Ξπ / Ξπ + π π]
    else
    {
      // delta_d_x = (((2 * sin) * (inertial_get_value / 2)) * (delta_S / delta_theta + sS));
      // delta_d_y = (((2 * sin) * (inertial_get_value / 2)) * (delta_R / delta_theta + sR));

      delta_d_x = 2 * okapi::sin(delta_theta / 2).getValue() * (delta_S / (delta_theta.convert(radian)) + sS);
      delta_d_y = 2 * okapi::sin(delta_theta / 2).getValue() * (delta_R / (delta_theta.convert(radian)) + sR);
    }

    //pros::lcd::print(4,"dt %5.1f ddx %5.1f ddy %5.1f",delta_theta.convert(degree), delta_d_x, delta_d_y);

    //9. Calculate the average orientation ππ = π0 + Ξπ / 2
    QAngle theta_m = theta_0 + delta_theta / 2;

    // printf("delta_d_x %6.4f delta_d_y %6.4f theta_m %6.4f\n",delta_d_x, delta_d_y, theta_m.convert(degree));

    // 10. Calculate global offset Ξπβ  as Ξπββ rotated by βππ; this can be done by converting your existing
    // Cartesian coordinates to polar coordinates, changing the angle, then converting back
    // 11. Calculate new absolute position π1βββ = π0 ββ + Ξπβ
    double cos_theta_m = okapi::cos(-theta_m).getValue();
	  double sin_theta_m = okapi::sin(-theta_m).getValue();

    // printf("cos_theta_m %6.4f sin_theta_m %6.4f\n",cos_theta_m, sin_theta_m);

  	// Update the global position
  	x_position += delta_d_x * cos_theta_m;
  	y_position += delta_d_x * sin_theta_m;

  	x_position += delta_d_y * -sin_theta_m; // -sin(x) = sin(-x)
  	y_position += delta_d_y * cos_theta_m; // cos(x) = cos(-x)

  	heading += delta_theta;
    theta_0 += delta_theta;

    pros::lcd::print(7,"X %5.1f Y %5.1f H %5.1f",x_position, y_position, heading.convert(degree));
    // printf("X %5.1f Y %5.1f H %5.1f\n",x_position, y_position, heading.convert(degree));

    pros::delay(10);
  }

}

double get_x_position()
{
  return x_position;
}

double get_y_position()
{
  return y_position;
}

QAngle get_heading()
{
  return heading;
}

//number, number, number_deg
//ex. reset_pos_generic(0.0, 0.0, 0.0_deg);
void reset_pos_generic(double new_x_pos, double new_y_pos)
// double reset_pos_generic()
{
  x_position = new_x_pos;
  y_position = new_y_pos;
  inertial_reset();
}

//starts the task that will read the location of the robot
void tracker_initialize()
{
  //uses the built-in pros task creator to start a task
  pros::Task position_tracker (position_tracker_task, (void*)"PROSV5", TASK_PRIORITY_MAX,
    TASK_STACK_DEPTH_DEFAULT, "Position Tracker Task");
}
