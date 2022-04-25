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
#include "position_tracker.hpp"
#include "math.h"
#include "drive_to_line.hpp"
using namespace okapi;

enum action { NOTHING, LIFT, INTAKE, LOWER_LIFTER };

struct point
{
  double x_pos;
  double y_pos;
  double speed;
  double m_t_s;
  double tr_kp;
  int action_1;
  int action_2;
  double a_1_p_1;
  double a_1_p_2;
  double a_2_p_1;
  double a_2_p_2;

  // action value;
  // int param1;
  // int param2;
};

void follow_path(point* points, int n_follow_points, double true_epsilon, double look_percent, bool point_or_line, bool reverse_drive);

const int max_points = 30;

point test_array[max_points];

point u_shape_array[max_points];

point right_side_rush_array[max_points];

point right_side_rush_rings_array[max_points];

point prog_skills_1_array[max_points];

point prog_skills_2_array[max_points];

point prog_skills_3_array[max_points];

void test_points()
{
  // follow_path(test_array, 6, 1, 20, false, false);
}

void u_shape()
{
  // u_shape_array[0].x_pos = 0.0;
  // u_shape_array[0].y_pos = 0.0;
  // u_shape_array[0].speed = 0.0;
  //
  // u_shape_array[1].x_pos = 12.0;
  // u_shape_array[1].y_pos = 52.0;
  // u_shape_array[1].speed = 80.0;
  //
  // u_shape_array[2].x_pos = 56.0;
  // u_shape_array[2].y_pos = 52.0;
  // u_shape_array[2].speed = 60.0;
  //
  // u_shape_array[3].x_pos = 68.0;
  // u_shape_array[3].y_pos = 0.0;
  // u_shape_array[3].speed = 80.0;
  //
  // follow_path(u_shape_array, 4, 1, 20, false, false);
}

void right_side_rush_backup()
{
  right_side_rush_array[0].x_pos = 0.0;
  right_side_rush_array[0].y_pos = 34.5;
  right_side_rush_array[0].speed = 0.0;
  right_side_rush_array[0].m_t_s = 60.0;
  right_side_rush_array[0].tr_kp = 0.01;
  right_side_rush_array[0].action_1 = NOTHING;
  right_side_rush_array[0].action_2 = NOTHING;

  right_side_rush_array[1].x_pos = 2.0;
  right_side_rush_array[1].y_pos = 33.0;
  right_side_rush_array[1].speed = 90.0;
  right_side_rush_array[1].m_t_s = 60.0;
  right_side_rush_array[1].tr_kp = 0.008;
  right_side_rush_array[1].action_1 = NOTHING;
  right_side_rush_array[1].action_2 = NOTHING;

  right_side_rush_array[2].x_pos = 16.5;
  right_side_rush_array[2].y_pos = 20.0;
  right_side_rush_array[2].speed = 80.0;
  right_side_rush_array[2].m_t_s = 60.0;
  right_side_rush_array[2].tr_kp = 0.008;
  right_side_rush_array[2].action_1 = NOTHING;
  right_side_rush_array[2].action_2 = NOTHING;

  follow_path(right_side_rush_array, 3, 2, 20, true, true);
}

void right_side_rush_rings()
{
//   right_side_rush_rings_array[0].x_pos = 16.0;
//   right_side_rush_rings_array[0].y_pos = 34.5;
//   right_side_rush_rings_array[0].speed = 0.0;
//   right_side_rush_rings_array[0].m_t_s = 60.0;
//   right_side_rush_rings_array[0].tr_kp = 0.01;
//   right_side_rush_rings_array[0].action_1 = NOTHING;
//   right_side_rush_rings_array[0].action_2 = NOTHING;
//
//   right_side_rush_rings_array[1].x_pos = 2.0;
//   right_side_rush_rings_array[1].y_pos = 33.0;
//   right_side_rush_rings_array[1].speed = 90.0;
//   right_side_rush_rings_array[1].m_t_s = 60.0;
//   right_side_rush_rings_array[1].tr_kp = 0.01;
//   right_side_rush_rings_array[1].action_1 = NOTHING;
//   right_side_rush_rings_array[1].action_2 = NOTHING;
//
//   right_side_rush_rings_array[2].x_pos = 16.5;
//   right_side_rush_rings_array[2].y_pos = 19.0;
//   right_side_rush_rings_array[2].speed = 80.0;
//   right_side_rush_rings_array[2].m_t_s = 60.0;
//   right_side_rush_rings_array[2].tr_kp = 0.01;
//   right_side_rush_rings_array[2].action_1 = NOTHING;
//   right_side_rush_rings_array[2].action_2 = NOTHING;
//
//   follow_path(right_side_rush_rings_array, 3, 2, 10, true, true);
}

void prog_skills_1()
{
  // prog_skills_1_array[0].x_pos = 0.0;
  // prog_skills_1_array[0].y_pos = -6;
  // prog_skills_1_array[0].speed = 0.0;
  //
  // prog_skills_1_array[1].x_pos = 8.0;
  // prog_skills_1_array[1].y_pos = 6.0;
  // prog_skills_1_array[1].speed = 60.0;
  //
  // prog_skills_1_array[2].x_pos = 28;
  // prog_skills_1_array[2].y_pos = 6.0;
  // prog_skills_1_array[2].speed = 60.0;
  //
  // prog_skills_1_array[3].x_pos = 30;
  // prog_skills_1_array[3].y_pos = -30.0;
  // prog_skills_1_array[3].speed = 40.0;
  //
  // prog_skills_1_array[4].x_pos = 36;
  // prog_skills_1_array[4].y_pos = -85.0;
  // prog_skills_1_array[4].speed = 40.0;

  prog_skills_1_array[0].x_pos = 0.0;
  prog_skills_1_array[0].y_pos = -6;
  prog_skills_1_array[0].speed = 0.0;
  prog_skills_1_array[0].m_t_s = 60.0;
  prog_skills_1_array[0].tr_kp = 0.01;
  prog_skills_1_array[0].action_1 = NOTHING;
  prog_skills_1_array[0].action_2 = NOTHING;

  prog_skills_1_array[1].x_pos = 8.0;
  prog_skills_1_array[1].y_pos = 6.0;
  prog_skills_1_array[1].speed = 40.0;
  prog_skills_1_array[1].m_t_s = 60.0;
  prog_skills_1_array[1].tr_kp = 0.01;
  prog_skills_1_array[1].action_1 = NOTHING;
  prog_skills_1_array[1].action_2 = NOTHING;

  prog_skills_1_array[2].x_pos = 26;
  prog_skills_1_array[2].y_pos = 6.0;
  prog_skills_1_array[2].speed = 40.0;
  prog_skills_1_array[2].m_t_s = 60.0;
  prog_skills_1_array[2].tr_kp = 0.01;
  prog_skills_1_array[2].action_1 = NOTHING;
  prog_skills_1_array[2].action_2 = NOTHING;

  prog_skills_1_array[3].x_pos = 30;
  prog_skills_1_array[3].y_pos = -30.0;
  prog_skills_1_array[3].speed = 35.0;
  prog_skills_1_array[3].m_t_s = 60.0;
  prog_skills_1_array[3].tr_kp = 0.01;
  prog_skills_1_array[3].action_1 = NOTHING;
  prog_skills_1_array[3].action_2 = NOTHING;

  prog_skills_1_array[4].x_pos = 38;
  prog_skills_1_array[4].y_pos = -72.0;
  prog_skills_1_array[4].speed = 35.0;
  prog_skills_1_array[4].m_t_s = 60.0;
  prog_skills_1_array[4].tr_kp = 0.01;
  prog_skills_1_array[4].action_1 = NOTHING;
  prog_skills_1_array[4].action_2 = NOTHING;

  prog_skills_1_array[5].x_pos = 38;
  prog_skills_1_array[5].y_pos = -85.0;
  prog_skills_1_array[5].speed = 35.0;
  prog_skills_1_array[5].m_t_s = 60.0;
  prog_skills_1_array[5].tr_kp = 0.01;
  prog_skills_1_array[5].action_1 = INTAKE;
  prog_skills_1_array[5].action_2 = LOWER_LIFTER;
  // LOWER_LIFTER
  // prog_skills_1_array[5].a_2_p_1 = 0.0;
  // prog_skills_1_array[5].a_2_p_2 = false;


  follow_path(prog_skills_1_array, 6, 1, 30, true, false);
}

void prog_skills_2()
{
  // prog_skills_2_array[0].x_pos = 29;
  // prog_skills_2_array[0].y_pos = -81;
  // prog_skills_2_array[0].speed = 0.0;
  //
  // prog_skills_2_array[1].x_pos = 43;
  // prog_skills_2_array[1].y_pos = -73;
  // prog_skills_2_array[1].speed = 50.0;
  //
  // prog_skills_2_array[2].x_pos = 54;
  // prog_skills_2_array[2].y_pos = -66;
  // prog_skills_2_array[2].speed = 25.0;
  //
  // prog_skills_2_array[3].x_pos = 66;
  // prog_skills_2_array[3].y_pos = -46;//-48
  // prog_skills_2_array[3].speed = 25.0;
  //
  // prog_skills_2_array[4].x_pos = 100;
  // prog_skills_2_array[4].y_pos = -42;
  // prog_skills_2_array[4].speed = 40.0;

  prog_skills_2_array[0].x_pos = 29;
  prog_skills_2_array[0].y_pos = -81;
  prog_skills_2_array[0].speed = 0.0;
  prog_skills_2_array[0].m_t_s = 100.0;
  prog_skills_2_array[0].tr_kp = 0.02;
  prog_skills_2_array[0].action_1 = NOTHING;
  prog_skills_2_array[0].action_2 = NOTHING;

  prog_skills_2_array[1].x_pos = 43;
  prog_skills_2_array[1].y_pos = -74.25;
  prog_skills_2_array[1].speed = 50.0;
  prog_skills_2_array[1].m_t_s = 100.0;
  prog_skills_2_array[1].tr_kp = 0.02;
  prog_skills_2_array[1].action_1 = NOTHING;
  prog_skills_2_array[1].action_2 = NOTHING;

  prog_skills_2_array[2].x_pos = 58;
  prog_skills_2_array[2].y_pos = -67;
  prog_skills_2_array[2].speed = 25.0;
  prog_skills_2_array[2].m_t_s = 100.0;
  prog_skills_2_array[2].tr_kp = 0.015;
  prog_skills_2_array[2].action_1 = NOTHING;
  prog_skills_2_array[2].action_2 = NOTHING;

  prog_skills_2_array[3].x_pos = 73;
  prog_skills_2_array[3].y_pos = -44;//-48
  prog_skills_2_array[3].speed = 25.0;
  prog_skills_2_array[3].m_t_s = 100.0;
  prog_skills_2_array[3].tr_kp = 0.02;
  prog_skills_2_array[3].action_1 = NOTHING;
  prog_skills_2_array[3].action_2 = NOTHING;

  prog_skills_2_array[4].x_pos = 101;
  prog_skills_2_array[4].y_pos = -39;
  prog_skills_2_array[4].speed = 20.0;
  prog_skills_2_array[4].m_t_s = 80.0;
  prog_skills_2_array[4].tr_kp = 0.015;
  prog_skills_2_array[4].action_1 = NOTHING;
  prog_skills_2_array[4].action_2 = NOTHING;

  follow_path(prog_skills_2_array, 5, 2, 10, true, false);
}

void prog_skills_3()
{
  prog_skills_3_array[0].x_pos = 66;
  prog_skills_3_array[0].y_pos = -12;
  prog_skills_3_array[0].speed = 0.0;
  prog_skills_3_array[0].m_t_s = 60.0;
  prog_skills_3_array[0].tr_kp = 0.02;
  prog_skills_3_array[0].action_1 = NOTHING;
  prog_skills_3_array[0].action_2 = NOTHING;

  prog_skills_3_array[1].x_pos = 74;
  prog_skills_3_array[1].y_pos = -20;
  prog_skills_3_array[1].speed = 70.0;
  prog_skills_3_array[1].m_t_s = 60.0;
  prog_skills_3_array[1].tr_kp = 0.02;
  prog_skills_3_array[1].action_1 = NOTHING;
  prog_skills_3_array[1].action_2 = NOTHING;

  prog_skills_3_array[2].x_pos = 99;
  prog_skills_3_array[2].y_pos = -37;
  prog_skills_3_array[2].speed = 70.0;
  prog_skills_3_array[2].m_t_s = 60.0;
  prog_skills_3_array[2].tr_kp = 0.01;
  prog_skills_3_array[2].action_1 = LOWER_LIFTER;
  prog_skills_3_array[2].action_2 = NOTHING;

  follow_path(prog_skills_3_array, 3, 2, 5, true, false);
}

void follow_path(point* points, int n_follow_points, double true_epsilon, double look_percent, bool point_or_line, bool reverse_drive = false)
{

  double current_x_position = get_x_position();
  double current_y_position = get_y_position();
  double current_heading = get_heading().convert(degree);
  int current_point = 0;

  // if((current_x_position > points[0].x_pos - 0.5 && current_x_position < points[0].x_pos + 0.5) &&
  //    (current_y_position > points[0].y_pos - 0.5 && current_y_position < points[0].y_pos + 0.5))
  // {

    //First point validated set target to next point
    current_point = current_point + 1;

    //turn to first point



    while(current_point < n_follow_points)
    {

      if(points[current_point].action_1 == LIFT)
      {
        move_arm(points[current_point].a_1_p_1, points[current_point].a_1_p_2);
      }
      else if(points[current_point].action_1 == INTAKE)
      {
        // async_intake(points[current_point].a_1_p_1, points[current_point].a_1_p_2);
        stop_intake();
      }
      else if(points[current_point].action_1 == LOWER_LIFTER)
      {
        lower_lifter();
      }
      if(points[current_point].action_2 == LIFT)
      {
        move_arm(points[current_point].a_2_p_1, points[current_point].a_2_p_2);
      }
      else if(points[current_point].action_2 == INTAKE)
      {
        // async_intake(points[current_point].a_2_p_1, points[current_point].a_2_p_2);
        stop_intake();
      }
      else if(points[current_point].action_2 == LOWER_LIFTER)
      {
        lower_lifter();
      }

      QLength x_targ = points[current_point].x_pos * inch;
      QLength y_targ = points[current_point].y_pos * inch;
      double targ_speed = points[current_point].speed;
      double m_turn_s = points[current_point].m_t_s;
      double turn_kp = points[current_point].tr_kp;

      if(current_point < n_follow_points - 1)
      {
        double x_lookahead = points[current_point + 1].x_pos;
        double y_lookahead = points[current_point + 1].y_pos;

        printf("drive_through %d\n", current_point);
        // drive_through_point(x_targ, y_targ, targ_speed, 60, 0.01, reverse_drive, 0.2, x_lookahead, y_lookahead, look_percent);
        // drive_through_point(x_targ, y_targ, targ_speed, 60, 0.01, reverse_drive, 0.4, x_lookahead, y_lookahead, look_percent);
        drive_through_point(x_targ, y_targ, targ_speed, m_turn_s, turn_kp, reverse_drive, 0.4, x_lookahead, y_lookahead, look_percent);
      }
      else
      {
        printf("drive_to %d\n", current_point);
        // drive_train.AutonomousArcadeDrive(0.0, 0.0);
        // break;
        if (point_or_line == true)
        {
          // curve_drive_to_point(x_targ, y_targ, targ_speed, 25, 0.03, -0.00416, 100, 0.008, 0.008, 200, 0, false, false, reverse_drive, NONE, 3, 0.00001, 2, 28, true_epsilon);
          // curve_drive_to_point(x_targ, y_targ, targ_speed, 25, 0.03, -0.00416, m_turn_s, turn_kp, 0.008, 200, 0, false, false, reverse_drive, NONE, 3, 0.00001, 2, 28, true_epsilon);
          curve_drive_to_point(x_targ, y_targ, targ_speed, 25, 0.03, -0.00416, m_turn_s, turn_kp, 0.008, 200, 0, false, false, reverse_drive, NONE, 3, 0.01, 2, 28, true_epsilon);

        }
        else
        {
          curve_drive_to_line(x_targ, y_targ, targ_speed, 25, 0.0208, -0.00416, 100, 0.008, reverse_drive, 0.2);
        }
      }

      current_point = current_point + 1;

    }
    printf("finished \n");
    printf("x_pos = %f y_pos = %f\n", get_x_position(),get_y_position());
  }

// }
