#include "okapi/api.hpp"
#include "inertial.hpp"
#include "normal_PID.hpp"
#include "initialize.hpp"
#include "vision.hpp"
#include "vision_drive.hpp"

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

double current_center = 0.0;
double current_bottom = 0.0;

//Drive X distance at Y speed
void vision_drive(int colour, double max_speed, double min_speed, double distance_to_mogo, double drive_straight_kp)
{
    //Chassis arcade takes values from -1 to 1 so this line allows a value from -100 to 100
    max_speed = max_speed / 100;
    min_speed = min_speed / 100;

    //Setting the Proportional,Integral,Differential constants (P.I.D.)
    const double drive_kp = 0.005;
    const double drive_ki = 0.0000;
    const double drive_kd = -0.0001;
    //Creates a constant for allowable error before stopping
    const double epsilon = 0.0;
    //Creates a maximum speed for velocity adjustment so that the robot will accelerate smoothly
    //and have no jerk at the beggining
    const double maximum_vel_adj = 0.1;
    //States the window in which the integral will be activated
    const double integral_limit = 250.0;
    //Sets the proportional constant for driving straight
    // const double drive_straight_kp = 0.0025;
    // const double drive_straight_kp = 0.003;
    
    double distance_from_bottom = 212 - current_bottom;
    //Draws starting position from the encoders (found in chassisController.cpp on github)
    double start_pos_val = shaft_enc_r->get_position() / 100.0;
    //Calculates current position based on start position (found in chassisController.cpp on github)
    double current_pos_val = (shaft_enc_r->get_position() / 100.0) - start_pos_val;
    //Sets last error to zero before driving starts
    double last_error = 0.0;

    double second_last_error = 9999.9;

    double third_last_error = 9999.9;

    double fourth_last_error = 9999.9;
    //Defines the initial drive error (found in chassisController.cpp on github)
    // double drive_error = distance_in_degrees - static_cast<double>((current_pos_values[0] + current_pos_values[1])) / 2.0;
    double drive_error = 212 - current_bottom;

    //Sets the first speed to zero
    double last_speed = drive_train.GetVelocity();

    double last_three_derivatives = 9999.9;

    double speed = 0.0;

    //Drive while the robot hasn't reached its target then glide last 11 inches
    while (drive_error > 0)
    {
        // ******************************************************************************************************************************* //
        //  This code uses proportional , differential, and integral constants to calculate the best speed to reach the desired distance   //
        // ******************************************************************************************************************************* //

        if(colour == RED_GOAL)
        {
          current_center = red_center;
          current_bottom = red_bottom;
        }

        if(colour == BLUE_GOAL)
        {
          current_center = blue_center;
          current_bottom = blue_bottom;
        }

        if(colour == YELLOW_GOAL)
        {
          current_center = yellow_center;
          current_bottom = yellow_bottom;
        }

        if ((current_center < 1.0) || (current_bottom < 1.0))
        {
          return;
        }

        //printf("Speed: %f  (p,i,d): (%f,%f,%f) ",speed,drive_error*drive_kp,integral*drive_ki,derivative*drive_kd);
        // printf("req speed: %f\n",speed);

        //printf("actual_robot_velocity = %7.3f\n", drive_train.GetVelocity());
        //printf("last three derivative = %7.3f\n", last_three_derivatives);
        //printf("drive error           = %7.3f\n", drive_error);

        //const int bottom_target = 212;
        const int bottom_target = 200;
        //Calculate distance left to drive
        drive_error = bottom_target - current_bottom;

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
        speed = drive_error * drive_kp + derivative * drive_kd;
        //printf("Speed: %f  (p,i,d): (%f,%f,%f) ",speed,drive_error*drive_kp,integral*drive_ki,derivative*drive_kd);

        //printf("middle_speed= %7.3f\n", speed);

        //Removes impossible speeds by setting the speed down to a possible one
        if(speed > max_speed)
        {
            speed = max_speed;
        }

        if(speed < max_speed * -1)
        {
            speed = max_speed * -1;
        }
        if((speed < min_speed) && (speed > 0.0))
        {
            speed = min_speed;
        }

        if((speed > min_speed * -1) && (speed < 0.0))
        {
            speed = min_speed * -1;
        }



        // ************************************************************************************************* //
        // Set maximum accelleration to prevent the robot from jerking right or left at the start of driving //
        // ************************************************************************************************* //

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
        // printf("adj speed: %f\n",speed);


        // ****************************************************************************************************************************** //
        // This code will make the robot drive straight by turning small distances if the robot has driven slightly to the right or left  //
        // - This does not support driving backwards right now.                                                                           //
        // ****************************************************************************************************************************** //
        double turn_speed = 0.0;

        const double final_target = 130;

        //Calculates the amount that the robot is off of its heading
        double drive_straight_error = final_target - current_center;

        //Creates a turn speed so that different sides can be slowed down
        turn_speed = drive_straight_error * drive_straight_kp;

        //Set a minimum turn speed.
        if (turn_speed > 0.01 && turn_speed < 0.040) turn_speed = 0.040;
        else if (turn_speed < -0.01 && turn_speed >- 0.040) turn_speed = -0.040;

        //Set a maximum turn speed
        if (turn_speed > 0.1) turn_speed = 0.1;
        else if (turn_speed < -0.1) turn_speed = -0.1;

        // ******************************************* //
        // Set final speed and calculate the new error //
        // ******************************************* //

        // printf("requested_velocity= %7.3f\n", speed);
        printf("curr: %f  error: %f, spd: %f\n",current_center,drive_straight_error,turn_speed);
        pros::lcd::print(3,"turn_speed %f  speed %f", turn_speed, speed);

        //Setting the desired speed in a percent form and waiting 10 milliseconds
        drive_train.AutonomousArcadeDrive(speed, turn_speed);
        pros::delay(33);

    }
    printf("STOP\n");

    drive_train.AutonomousArcadeDrive(speed, 0.0);

    // Calculate 11 inches on encoder wheel.
    start_pos_val = shaft_enc_r->get_position() / 100.0;

    current_pos_val = (shaft_enc_r->get_position() / 100.0) - start_pos_val;

    double distance_travelled = current_pos_val / degrees_per_inch;
    printf("initial_dist_trav: %f\n",distance_travelled);

    // while distance < 11 keep current speeds
    while(distance_travelled < distance_to_mogo)
    {
      printf("dist_trav: %f\n",distance_travelled);

      current_pos_val = (shaft_enc_r->get_position() / 100.0) - start_pos_val;
      distance_travelled = current_pos_val / degrees_per_inch;
      pros::delay(10);
    }
    // stop
    drive_train.AutonomousArcadeDrive(0.0, 0.0);
    // printf("STOP\n");
    // while(true){
    //   current_pos_val = (shaft_enc_r->get_position() / 100.0) - start_pos_val;
    //   distance_travelled = current_pos_val / degrees_per_inch;
    //   printf("final_dist_trav: %f\n",distance_travelled);
    //   pros::delay(10);
    // }

    //Stops the robot from moving after the robot has reached its target distance
    // drive_train.AutonomousArcadeDrive(0.0, 0.0);
}

// QLength async_distance;
// double async_max_pos_speed;
// bool async_complete = true;
// pros::Task* drive_task = NULL;
//
// void drive_async(void* param)
// {
//   while (true)
//   {
//     if(!async_complete)
//     {
//       //gyro_drive(async_chassis, async_distance, async_max_speed, async_kp, async_ki, async_kd);
//       gyro_drive(async_distance, async_max_pos_speed);
//       async_complete = true;
//     }
//     pros::delay(33);
//   }
// }
//
// bool is_drive_complete()
// {
//   return async_complete;
// }
//
// void wait_for_drive_complete()
// {
//   while(!async_complete)
//   {
//     pros::delay(10);
//   }
// }
//
// //void async_gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed, double kp, double ki, double kd)
// void async_gyro_drive(std::shared_ptr<ChassisController> chassis, QLength distance, double max_speed)
// {
//   async_distance = distance;
//   async_max_pos_speed = max_speed;
//   // async_kp = kp;
//   // async_ki = ki;
//   // async_kd = kd;
//
//   if (drive_task == NULL)
//   {
//     drive_task = new pros::Task(drive_async, (void*)"PROSDRIVE", TASK_PRIORITY_DEFAULT,
//                                              TASK_STACK_DEPTH_DEFAULT, "Async Drive Task");
//   }
//
//   async_complete = false;
// }
