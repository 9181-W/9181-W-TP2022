#include "okapi/api.hpp"
#include "inertial.hpp"
#include "normal_PID.hpp"
#include "initialize.hpp"

using namespace okapi;

//Creates a constant for wheel diameter
const double wheel_diam = 2.75;
//Creates a constant for pi
const double drive_pi = 3.14159265359;
//Calculates a constant wheel circumference using diameter and pi
const double wheel_circ = wheel_diam * drive_pi;
//Encoder degrees in circumference
const double degrees_per_circ = 360.0;
//Encoder degrees per inch
const double degrees_per_inch = degrees_per_circ / wheel_circ;

//Drive X distance at Y speed
void gyro_drive(QLength distance, double drive_kp, double max_speed, double min_speed, bool drive_straight, double drive_straight_kp, double epsilon)
{

    //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
    max_speed = max_speed / 100;
    min_speed = min_speed / 100;

    //Setting the Proportional,Integral,Differential constants (P.I.D.)
    // const double drive_kp = 0.0005;
    const double drive_ki = 0.0000;
    const double drive_kd = -0.0001;
    //Creates a constant for allowable error before stopping
    // const double epsilon = 0.5;
    const double zero_speed = 0.05;
    //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
    //and have no jerk at the beggining
    const double maximum_vel_adj = 0.1;
    //States the window in which the integral will be activated
    const double integral_limit = 250.0;
    //Sets the proportional constant for driving straight
    // const double drive_straight_kp = 0.03;
    // const double drive_straight_kp = 0.01;

    //Converts Qlength distance to distance_in_inches
    double distance_in_inches = distance.convert(inch);
    //Convert distance to encoder degrees
    const double distance_in_degrees = distance_in_inches * degrees_per_inch;
    //Draws starting position from the encoders (found in chassisController.cpp on github)
    double start_pos_val = shaft_enc_r->get_position() / 100.0;
    //Calculates current position based on start position (found in chassisController.cpp on github)
    double current_pos_val = (shaft_enc_r->get_position() / 100.0) - start_pos_val;
    //Sets the integral to zero so that additions can be made latetr
    double integral = 0.0;
    //Sets last error to zero before driving starts
    double last_error = 0.0;

    double second_last_error = 9999.9;

    double third_last_error = 9999.9;

    double fourth_last_error = 9999.9;
    //Defines the initial drive error (found in chassisController.cpp on github)
    // double drive_error = distance_in_degrees - static_cast<double>((current_pos_values[0] + current_pos_values[1])) / 2.0;
    double drive_error = distance_in_degrees - current_pos_val;
    //Creates a variable that contains the initial gyro value (0)
    inertial_reset();
    double initial_drive_gyro_value = inertial_get_value();

    //Sets the first speed to zero
    double last_speed = 0.0;

    double last_three_derivatives = 9999.9;

    //Drive while the robot hasn't reached its target distance
    // while ((fabs(last_three_derivatives) > epsilon) || (fabs(drive_error) > fabs(distance_in_degrees) / 2.0))
    while(drive_error > 1.0)
    //while (fabs(last_three_derivatives) > epsilon)
    {
        // ******************************************************************************************************************************* //
        //  This code uses proportional , differential, and integral constants to calculate the best speed to reach the desired distance   //
        // ******************************************************************************************************************************* //

        //printf("Speed: %f  (p,i,d): (%f,%f,%f) ",speed,drive_error*drive_kp,integral*drive_ki,derivative*drive_kd);
        // printf("req speed: %f\n",speed);

        // printf("actual_robot_velocity = %7.3f\n", drive_train.GetVelocity());
        // printf("last three derivative = %7.3f\n", last_three_derivatives);
        // printf("drive error           = %7.3f\n", drive_error);

        //Calculate distance left to drive
        // drive_error = distance_in_degrees - static_cast<double>((current_pos_values[0] + current_pos_values[1])) / 4.0;;
        drive_error = distance_in_degrees - current_pos_val;

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

        //Determines when the integral will start being used
        if(drive_ki != 0)
        {
            if(fabs(drive_error) < integral_limit)
            {
                integral = integral + drive_error;
            }
            else
            {
                integral = 0;
            }
        }

        if((integral > 0) && (drive_error < 0))
        {
          integral = integral * -1;
        }
        else if((integral < 0) && (drive_error > 0))
        {
          integral = integral * -1;
        }

        //Calculate speed to be driven at using kp,ki,kd
        double speed = drive_error * drive_kp + integral * drive_ki + derivative * drive_kd;
        //printf("Speed: %f  (p,i,d): (%f,%f,%f) ",speed,drive_error*drive_kp,integral*drive_ki,derivative*drive_kd);

        // printf("middle_speed= %7.3f\n", speed);

        //Removes impossible speeds by setting the speed down to a possible one
        if(speed > max_speed)
        {
            speed = max_speed;
        }

        if(speed < max_speed * -1)
        {
            speed = max_speed * -1;
        }

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
            // speed = last_speed + maximum_vel_adj;
            speed = speed;
        }

         if(velocity_adj < -maximum_vel_adj)
        {
            // speed = last_speed + -maximum_vel_adj;
            speed = speed;
        }

        last_speed = speed;
        // printf("adj speed: %f\n",speed);


        // ****************************************************************************************************************************** //
        // This code will make the robot drive straight by turning small distances if the robot has driven slightly to the right or left  //
        // - This does not support driving backwards right now.                                                                           //
        // ****************************************************************************************************************************** //
        double turn_speed = 0.0;
        if(drive_straight == true)
        {
          //Gets the gyro's current value
          double drive_gyro_value = inertial_get_value();

          //Calculates the amount that the robot is off of its heading
          double drive_straight_error = initial_drive_gyro_value - drive_gyro_value;

          printf("gyro target %7.3f  gyro: %7.3f  ", initial_drive_gyro_value, drive_gyro_value);

          //Creates a turn speed so that different sides can be slowed down
          turn_speed = drive_straight_error * drive_straight_kp;
        }

        // ******************************************* //
        // Set final speed and calculate the new error //
        // ******************************************* //

        printf("speed= %7.3f  turn_speed= %7.3f\n", speed, -turn_speed);

        //Setting the desired speed in a percent form and waiting 10 milliseconds
        // if(drive_error > (distance_in_degrees * 0.25))
        // {
          drive_train.AutonomousArcadeDrive(speed, -turn_speed, false);
          // drive_train.AutonomousArcadeDrive(speed, 0.0, false);

        // }
        // else
        // {
        //   drive_train.AutonomousArcadeDrive(speed, -turn_speed);
        //   // drive_train.AutonomousArcadeDrive(speed, 0.0);
        //
        // }
        pros::delay(33);

        //Calculates current position based on start position after small movement
        // current_pos_values = get_sensor_vals(chassis) - start_pos_values;
        current_pos_val = (shaft_enc_r->get_position() / 100.0) - start_pos_val;

    }

    //Stops the robot from moving after the robot has reached its target distance
    drive_train.AutonomousArcadeDrive(0.0, 0.0);
}

QLength async_distance_1;
double async_max_pos_speed_1;
double async_drive_kp_1;
double async_min_speed_1;
bool async_drive_straight_1;
double async_drive_straight_kp_1;
double async_epsilon_1;
bool async_complete = true;
pros::Task* drive_task = NULL;

void drive_async(void* param)
{
  while (true)
  {
    if(!async_complete)
    {
      //gyro_drive(async_chassis, async_distance, async_max_speed, async_kp, async_ki, async_kd);
      gyro_drive(async_distance_1, async_drive_kp_1, async_max_pos_speed_1, async_min_speed_1, async_drive_straight_1, async_drive_straight_kp_1, async_epsilon_1);
      async_complete = true;
    }
    pros::delay(33);
  }
}

bool is_drive_complete()
{
  return async_complete;
}

void wait_for_drive_complete()
{
  while(!async_complete)
  {
    pros::delay(33);
  }
}

//void async_gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed, double kp, double ki, double kd)
void async_gyro_drive(QLength distance, double drive_kp, double max_speed, double min_speed, bool drive_straight, double drive_straight_kp, double epsilon)
{
  async_distance_1 = distance;
  async_drive_kp_1 = drive_kp;
  async_max_pos_speed_1 = max_speed;
  async_min_speed_1 = min_speed;
  async_drive_straight_1 = drive_straight;
  async_drive_straight_kp_1 = drive_straight_kp;
  async_epsilon_1 = epsilon;
  // async_kp = kp;
  // async_ki = ki;
  // async_kd = kd;

  if (drive_task == NULL)
  {
    drive_task = new pros::Task(drive_async, (void*)"PROSDRIVE", TASK_PRIORITY_DEFAULT,
                                             TASK_STACK_DEPTH_DEFAULT, "Async Drive Task");
  }

  async_complete = false;
}
