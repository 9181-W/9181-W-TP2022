#include "okapi/api.hpp"
#include "drive_train.hpp"
#include "initialize.hpp"

bool autonomous_on = false;
double requested_left_vel = 0.0;
double requested_right_vel = 0.0;

void left_vel_pid(void* param)
{
  //Constants//
  const double kp = 0.5;
  const double ki = 0.0;
  const double kd = 0.3;

  //PID Variables Here//
  double current_velocity = drive_train.GetLeftVelocity();
  double error = requested_left_vel - current_velocity;
  double last_error = 0.0;
  // double total_error = 0.0;
  // double integral_active_zone = 0.15;
  double current_voltage = drive_train.GetLeftVoltage();

  // double on_target_count = 0.0;
  double final_adjustment = error * kp; //add the rest of PID to this calculation

  //Temp Variable//
  double delta_time = 0.0;

  while(true)
  {
    if (autonomous_on)
    {
      if (fabs(requested_left_vel) < 0.05)
      {
        pros::lcd::print(5,"Exit via speed");
        drive_train.MoveLeftVoltage(0.0);
      }
      else
      {
        pros::lcd::print(4,"requested_left_vel %f", requested_left_vel);
        current_voltage = drive_train.GetLeftVoltage();
    		current_velocity = drive_train.GetLeftVelocity();
    		error = requested_left_vel - current_velocity;

    		if (error == 0.0)
    		{
    			last_error = 0.0;
    		}

    		// if (abs(error) < integral_active_zone && error != 0.0)
    		// {
    		// 	total_error += error;
    		// }
    		// else
    		// {
    		// 	total_error = 0.0;
    		// }

    		// final_adjustment = ((error * kp) + (total_error * ki) + ((error - last_error) * kd));
        final_adjustment = ((error * kp) + ((error - last_error) * kd));
    		current_voltage += final_adjustment;
    		if (current_voltage > 1.0)
    		{
    			current_voltage = 1.0;
    		}
    		else if (current_voltage < -1.0)
    		{
    			current_voltage = -1.0;
    		}

    		drive_train.MoveLeftVoltage(current_voltage);

        last_error = error;
      }
    }
    pros::delay(10);
  }
}

void right_vel_pid(void* param)
{
  //Constants//
  const double kp = 0.5;
  const double ki = 0.0;
  const double kd = 0.30;

  //PID Variables Here//
  double current_velocity = drive_train.GetRightVelocity();
  double error = requested_right_vel - current_velocity;
  double last_error = 0.0;
  // double total_error = 0.0;
  // double integral_active_zone = 0.15;
  double current_voltage = drive_train.GetRightVoltage();

  // double on_target_count = 0.0;
  double final_adjustment = error * kp; //add the rest of PID to this calculation

  //Temp Variable//
  double delta_time = 0.0;

  while(true)
  {
    if (autonomous_on)
    {
      if (fabs(requested_right_vel) < 0.05)
      {
        drive_train.MoveRightVoltage(0.0);
      }
      else
      {
        current_voltage = drive_train.GetRightVoltage();
    		current_velocity = drive_train.GetRightVelocity();
        //printf("request: %7.3f  current: %7.3f (%7.3f)\n",requested_right_vel,current_velocity,current_voltage);
    		error = requested_right_vel - current_velocity;

    		if (error == 0.0)
    		{
    			last_error = 0.0;
    		}

    		// if (abs(error) < integral_active_zone && error != 0.0)
    		// {
    		// 	total_error += error;
    		// }
    		// else
    		// {
    		// 	total_error = 0.0;
    		// }

    		// final_adjustment = ((error * kp) + (total_error * ki) + ((error - last_error) * kd));
        final_adjustment = ((error * kp) + ((error - last_error) * kd));
        //printf("adf: %7.3f  error: %7.3f  prop: %7.3f  diff: %7.3f)\n",final_adjustment,error,error * kp,((error - last_error) * kd));
    		current_voltage += final_adjustment;
    		if (current_voltage > 1.0)
    		{
    			current_voltage = 1.0;
    		}
    		else if (current_voltage < -1.0)
    		{
    			current_voltage = -1.0;
    		}

    		drive_train.MoveRightVoltage(current_voltage);

        last_error = error;
      }
    }
    pros::delay(16);
  }
}

DriveTrain::DriveTrain(MotorGroup left, MotorGroup right,
           AbstractMotor::gearset geartype,
           AbstractMotor::encoderUnits units,
           AbstractMotor::brakeMode braketype) :
   m_LeftMotorGroup(left),
   m_RightMotorGroup(right),
   m_GearType(geartype),
   m_Units(units),
   m_BrakeType(braketype),
   m_left_vel_pid_task(NULL),
   m_right_vel_pid_task(NULL)
{
  m_LeftMotorGroup.setGearing(geartype);
  m_RightMotorGroup.setGearing(geartype);
  m_LeftMotorGroup.setEncoderUnits(units);
  m_RightMotorGroup.setEncoderUnits(units);
  m_LeftMotorGroup.setBrakeMode(braketype);
  m_RightMotorGroup.setBrakeMode(braketype);
  m_left_vel_pid_task = new pros::Task(left_vel_pid, (void*)"PROSLVELPID", TASK_PRIORITY_DEFAULT,
                                           TASK_STACK_DEPTH_DEFAULT, "Left Vel Pid Task");
  m_right_vel_pid_task = new pros::Task(right_vel_pid, (void*)"PROSRVELPID", TASK_PRIORITY_DEFAULT,
                                           TASK_STACK_DEPTH_DEFAULT, "Right Vel Pid Task");
}


void DriveTrain::ArcadeDrive(double drive_speed, double turn_speed)
{
  autonomous_on = false;

  // Parameter Validation
  drive_speed = std::clamp(drive_speed, -1.0, 1.0);
  turn_speed = std::clamp(turn_speed, -1.0, 1.0);

  double left_speed = drive_speed - turn_speed;
  double right_speed = drive_speed + turn_speed;

  left_speed = std::clamp(left_speed, -1.0, 1.0);
  right_speed = std::clamp(right_speed, -1.0, 1.0);

  double gearset_rpm = 200.0;
  if (m_GearType == AbstractMotor::gearset::blue) gearset_rpm = 600.0;
  if (m_GearType == AbstractMotor::gearset::red) gearset_rpm = 100.0;

  left_speed *= gearset_rpm;
  right_speed *= gearset_rpm;

  // left_speed *= 12000;
  // right_speed *= 12000;

  m_LeftMotorGroup.moveVelocity(left_speed);
  m_RightMotorGroup.moveVelocity(right_speed);
  // m_LeftMotorGroup.moveVoltage(left_speed);
  // m_RightMotorGroup.moveVoltage(right_speed);
}

void DriveTrain::AutonomousArcadeDrive(double drive_speed, double turn_speed, bool velocity_control)
{
  autonomous_on = velocity_control;
  // if (m_left_vel_pid_task == NULL)
  // {
  //   m_left_vel_pid_task = new pros::Task(left_vel_pid, (void*)"PROSLVELPID", TASK_PRIORITY_DEFAULT,
  //                                            TASK_STACK_DEPTH_DEFAULT, "Left Vel Pid Task");
  // }
  // if (m_right_vel_pid_task == NULL)
  // {
  //   m_right_vel_pid_task = new pros::Task(right_vel_pid, (void*)"PROSLVELPID", TASK_PRIORITY_DEFAULT,
  //                                            TASK_STACK_DEPTH_DEFAULT, "Right Vel Pid Task");
  // }

  // Parameter Validation
  drive_speed = std::clamp(drive_speed, -1.0, 1.0);
  turn_speed = std::clamp(turn_speed, -1.0, 1.0);

  double left_speed = drive_speed - turn_speed;
  double right_speed = drive_speed + turn_speed;

  left_speed = std::clamp(left_speed, -1.0, 1.0);
  right_speed = std::clamp(right_speed, -1.0, 1.0);

  printf("2 left_speed %8.3f  right_speed %8.3f\n", left_speed, right_speed);
  requested_left_vel = left_speed;
  requested_right_vel = right_speed;

  if (velocity_control == false)
  {
    m_LeftMotorGroup.moveVoltage(left_speed*12000);
    m_RightMotorGroup.moveVoltage(right_speed*12000);
    // m_LeftMotorGroup.moveVelocity(left_speed * 200);
    // m_RightMotorGroup.moveVelocity(right_speed * 200);
    // m_LeftMotorGroup.moveVelocity(200);
    // m_RightMotorGroup.moveVelocity(200);

  }
}

double DriveTrain::GetVelocity()
{
  double gearset_rpm = 200.0;
  if (m_GearType == AbstractMotor::gearset::blue) gearset_rpm = 600.0;
  if (m_GearType == AbstractMotor::gearset::red) gearset_rpm = 100.0;

  double velocity = m_LeftMotorGroup.getActualVelocity() + m_RightMotorGroup.getActualVelocity();
  velocity = velocity / 2.0;
  velocity = velocity / gearset_rpm;

  return velocity;
}

double DriveTrain::GetLeftVelocity()
{
  double gearset_rpm = 200.0;
  if (m_GearType == AbstractMotor::gearset::blue) gearset_rpm = 600.0;
  if (m_GearType == AbstractMotor::gearset::red) gearset_rpm = 100.0;

  double velocity = m_LeftMotorGroup.getActualVelocity();
  velocity = velocity / gearset_rpm;

  return velocity;
}

double DriveTrain::GetRightVelocity()
{
  double gearset_rpm = 200.0;
  if (m_GearType == AbstractMotor::gearset::blue) gearset_rpm = 600.0;
  if (m_GearType == AbstractMotor::gearset::red) gearset_rpm = 100.0;

  double velocity = m_RightMotorGroup.getActualVelocity();
  velocity = velocity / gearset_rpm;

  return velocity;
}

double DriveTrain::GetLeftVoltage()
{
  return (m_LeftMotorGroup.getVoltage() / 12000.0);
}

double DriveTrain::GetRightVoltage()
{
  return (m_RightMotorGroup.getVoltage() / 12000.0);
}

void DriveTrain::MoveLeftVoltage(double speed)
{
  printf("move left voltage %8.2f\n", (speed * 12000));
  //if the speed is below a certain value set it to a moveVelocity so as to respect the brake mode
  if(fabs(speed) < 0.03)
  {
    m_LeftMotorGroup.moveVelocity(0.0);
  }
  else
  {
    m_LeftMotorGroup.moveVoltage(speed * 12000);
  }
}

void DriveTrain::MoveRightVoltage(double speed)
{
  printf("move right voltage %8.2f\n", (speed * 12000));
  // if the speed is below a certain value set it to a moveVelocity so as to respect the brake mode
  if(fabs(speed) < 0.03)
  {
    m_RightMotorGroup.moveVelocity(0.0);
  }
  else
  {
    m_RightMotorGroup.moveVoltage(speed * 12000);
  }
}

void DriveTrain::MoveVelocity(double drive_speed)
{
  // double gearset_rpm = 200.0;
  // if (m_GearType == AbstractMotor::gearset::blue) gearset_rpm = 600.0;
  // if (m_GearType == AbstractMotor::gearset::red) gearset_rpm = 100.0;
  //
  // double velocity = drive_speed + gearset_rpm;
  //
  // m_LeftMotorGroup.setGearing(AbstractMotor::gearset::green);
  // m_RightMotorGroup.setGearing(AbstractMotor::gearset::blue);
  // m_LeftMotorGroup.moveVelocity(100);
  // m_RightMotorGroup.moveVelocity(100);

  requested_left_vel = drive_speed;
  requested_right_vel = drive_speed;
}

void DriveTrain::SetBrakeMode(AbstractMotor::brakeMode braketype)
{
  m_BrakeType = braketype;
  m_LeftMotorGroup.setBrakeMode(braketype);
  m_RightMotorGroup.setBrakeMode(braketype);
}
