#include "okapi/api.hpp"
using namespace okapi;

#pragma once

class DriveTrain
{
  public:
    DriveTrain(MotorGroup left, MotorGroup right,
               AbstractMotor::gearset geartype = AbstractMotor::gearset::green,
               AbstractMotor::encoderUnits units = AbstractMotor::encoderUnits::degrees,
               AbstractMotor::brakeMode braketype = AbstractMotor::brakeMode::coast);

    void ArcadeDrive(double drive_speed, double turn_speed);
    void AutonomousArcadeDrive(double drive_speed, double turn_speed);
    double GetVelocity();
    double GetLeftVelocity();
    double GetRightVelocity();
    double GetLeftVoltage();
    double GetRightVoltage();
    void MoveLeftVoltage(double speed);
    void MoveRightVoltage(double speed);
    void SetBrakeMode(AbstractMotor::brakeMode braketype);
    void MoveVelocity(double drive_speed);

  private:
    MotorGroup m_LeftMotorGroup;
    MotorGroup m_RightMotorGroup;
    AbstractMotor::gearset m_GearType;
    AbstractMotor::encoderUnits m_Units;
    AbstractMotor::brakeMode m_BrakeType;
    pros::Task* m_left_vel_pid_task;
    pros::Task* m_right_vel_pid_task;
};
