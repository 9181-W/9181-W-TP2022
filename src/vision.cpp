#include "main.h"
#include "okapi/api.hpp"
#include "drive_train.hpp"
#include "opcontrol.hpp"
#include "initialize.hpp"
#include "normal_PID.hpp"
#include "curve_drive.hpp"
#include "utils.hpp"
#include "vision.hpp"

pros::Vision vision_sensor (VISION_PORT);

pros::vision_signature_s_t red_1;
pros::vision_signature_s_t blue_1;
pros::vision_signature_s_t yellow_1;
int red_area = 0;
int red_center = 0;
int red_bottom = 0;
int blue_area = 0;
int blue_center = 0;
int blue_bottom = 0;
int yellow_area = 0;
int yellow_center = 0;
int yellow_bottom = 0;

extern int errno ;

// typedef struct __attribute__((__packed__)) vision_object {
// // Object signature
// uint16_t signature;
// // Object type, e.g. normal, color code, or line detection
// vision_object_type_e_t type;
// // left boundary coordinate of the object
// uint16_t left_coord;
// // top boundary coordinate of the object
// uint16_t top_coord;
// // width of the object
// uint16_t width;
// // height of the object
// uint16_t height;
// // Angle of a color code object in 0.1 degree units (e.g. 10 -> 1 degree, 155 -> 15.5 degrees)
// uint16_t angle;
//
// // coordinates of the middle of the object (computed from the values above)
// uint16_t x_middle_coord;
// uint16_t y_middle_coord;
// } vision_object_s_t;

void detect_mobile_goals(void* param)
{
  const int average_size = 10;

  // Blue Goal ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int blue_l[average_size];
  int blue_l_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    blue_l[i] = 0;
  }

  int blue_t[average_size];
  int blue_t_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    blue_t[i] = 0;
  }

  int blue_w[average_size];
  int blue_w_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    blue_w[i] = 0;
  }

  int blue_h[average_size];
  int blue_h_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    blue_h[i] = 0;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  // Red Goal //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int red_l[average_size];
  int red_l_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    red_l[i] = 0;
  }

  int red_t[average_size];
  int red_t_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    red_t[i] = 0;
  }

  int red_w[average_size];
  int red_w_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    red_w[i] = 0;
  }

  int red_h[average_size];
  int red_h_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    red_h[i] = 0;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



  // Yellow Goal ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  int yellow_l[average_size];
  int yellow_l_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    yellow_l[i] = 0;
  }

  int yellow_t[average_size];
  int yellow_t_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    yellow_t[i] = 0;
  }

  int yellow_w[average_size];
  int yellow_w_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    yellow_w[i] = 0;
  }

  int yellow_h[average_size];
  int yellow_h_avg = 0;
  for (int i=0; i<average_size; i++)
  {
    yellow_h[i] = 0;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  while(true)
  {
    // Blue Goal ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pros::vision_object_s_t blue = vision_sensor.get_by_sig(0, blue_1.id);
    pros::vision_object_s_t blue_object_arr[7];
    std::int32_t blue_count = vision_sensor.read_by_sig(0, blue_1.id, 7,blue_object_arr);

    if (blue_count < 1)
    {
      blue.left_coord = 0;
      blue.top_coord = 0;
      blue.width = 0;
      blue.height = 0;
      //pros::lcd::print(2,"no red boxes");
    }
    else if ((blue.left_coord > 316) || (blue.left_coord < 0) ||
              (blue.top_coord > 316) || (blue.top_coord < 0) ||
              (blue.width > 316) || (blue.width < 0) ||
              (blue.height > 316) || (blue.height < 0))
    {
        blue.left_coord = 0;
        blue.top_coord = 0;
        blue.width = 0;
        blue.height = 0;
    }
    else if (blue.left_coord > 0)
    {
      // Add to average array of size 4.
      for (int i=0; i<average_size-1; i++)
      {
        blue_l[i] = blue_l[i+1];
      }
      blue_l[average_size-1] = blue.left_coord;
      for (int i=0; i<average_size-1; i++)
      {
        blue_t[i] = blue_t[i+1];
      }
      blue_t[average_size-1] = blue.top_coord;
      for (int i=0; i<average_size-1; i++)
      {
        blue_w[i] = blue_w[i+1];
      }
      blue_w[average_size-1] = blue.width;
      for (int i=0; i<average_size-1; i++)
      {
        blue_h[i] = blue_h[i+1];
      }
      blue_h[average_size-1] = blue.height;

      // Calculate average
      blue_l_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        blue_l_avg += blue_l[i];
      }
      blue_l_avg = blue_l_avg/average_size;
      blue_t_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        blue_t_avg += blue_t[i];
      }
      blue_t_avg = blue_t_avg/average_size;
      blue_w_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        blue_w_avg += blue_w[i];
      }
      blue_w_avg = blue_w_avg/average_size;
      blue_h_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        blue_h_avg += blue_h[i];
      }
      blue_h_avg = blue_h_avg/average_size;
      //pros::lcd::print(2,"red box %3d,%3d,%3d,%3d", red_l_avg, red_t_avg, red_w_avg, red_h_avg);
    }
    blue_area = blue_w_avg*blue_h_avg;
    blue_center = blue_l_avg+blue_w_avg/2;
    blue_bottom = blue_t_avg+blue_h_avg;
    // pros::lcd::print(2,"blue centre %d  blue bottom %d", blue_center, blue_bottom);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    // Red Goal ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pros::vision_object_s_t red = vision_sensor.get_by_sig(0, red_1.id);
    pros::vision_object_s_t red_object_arr[7];
    std::int32_t red_count = vision_sensor.read_by_sig(0, red_1.id, 7,red_object_arr);

    if (red_count < 1)
    {
      red.left_coord = 0;
      red.top_coord = 0;
      red.width = 0;
      red.height = 0;
      //pros::lcd::print(2,"no red boxes");
    }
    else if ((red.left_coord > 316) || (red.left_coord < 0) ||
              (red.top_coord > 316) || (red.top_coord < 0) ||
              (red.width > 316) || (red.width < 0) ||
              (red.height > 316) || (red.height < 0))
    {
        red.left_coord = 0;
        red.top_coord = 0;
        red.width = 0;
        red.height = 0;
    }
    else if (red.left_coord > 0)
    {
      // Add to average array of size 4.
      for (int i=0; i<average_size-1; i++)
      {
        red_l[i] = red_l[i+1];
      }
      red_l[average_size-1] = red.left_coord;
      for (int i=0; i<average_size-1; i++)
      {
        red_t[i] = red_t[i+1];
      }
      red_t[average_size-1] = red.top_coord;
      for (int i=0; i<average_size-1; i++)
      {
        red_w[i] = red_w[i+1];
      }
      red_w[average_size-1] = red.width;
      for (int i=0; i<average_size-1; i++)
      {
        red_h[i] = red_h[i+1];
      }
      red_h[average_size-1] = red.height;

      // Calculate average
      red_l_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        red_l_avg += red_l[i];
      }
      red_l_avg = red_l_avg/average_size;
      red_t_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        red_t_avg += red_t[i];
      }
      red_t_avg = red_t_avg/average_size;
      red_w_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        red_w_avg += red_w[i];
      }
      red_w_avg = red_w_avg/average_size;
      red_h_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        red_h_avg += red_h[i];
      }
      red_h_avg = red_h_avg/average_size;
      // pros::lcd::print(2,"red box %3d,%3d,%3d,%3d", red_l_avg, red_t_avg, red_w_avg, red_h_avg);
    }
    red_area = red_w_avg*red_h_avg;
    red_center = red_l_avg+red_w_avg/2;
    red_bottom = red_t_avg+red_h_avg;
    pros::lcd::print(2,"red centre %d  red bottom %d", red_center, red_bottom);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    // Yellow Goal //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pros::vision_object_s_t yellow = vision_sensor.get_by_sig(0, yellow_1.id);
    pros::vision_object_s_t yellow_object_arr[7];
    std::int32_t yellow_count = vision_sensor.read_by_sig(0, yellow_1.id, 7,yellow_object_arr);

    if (yellow_count < 1)
    {
      yellow.left_coord = 0;
      yellow.top_coord = 0;
      yellow.width = 0;
      yellow.height = 0;
      //pros::lcd::print(2,"no red boxes");
    }
    else if ((yellow.left_coord > 316) || (yellow.left_coord < 0) ||
              (yellow.top_coord > 316) || (yellow.top_coord < 0) ||
              (yellow.width > 316) || (yellow.width < 0) ||
              (yellow.height > 316) || (yellow.height < 0))
    {
        yellow.left_coord = 0;
        yellow.top_coord = 0;
        yellow.width = 0;
        yellow.height = 0;
    }
    else if (yellow.left_coord > 0)
    {
      // Add to average array of size 4.
      for (int i=0; i<average_size-1; i++)
      {
        yellow_l[i] = yellow_l[i+1];
      }
      yellow_l[average_size-1] = yellow.left_coord;
      for (int i=0; i<average_size-1; i++)
      {
        yellow_t[i] = yellow_t[i+1];
      }
      yellow_t[average_size-1] = yellow.top_coord;
      for (int i=0; i<average_size-1; i++)
      {
        yellow_w[i] = yellow_w[i+1];
      }
      yellow_w[average_size-1] = yellow.width;
      for (int i=0; i<average_size-1; i++)
      {
        yellow_h[i] = yellow_h[i+1];
      }
      yellow_h[average_size-1] = yellow.height;

      // Calculate average
      yellow_l_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        yellow_l_avg += yellow_l[i];
      }
      yellow_l_avg = yellow_l_avg/average_size;
      yellow_t_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        yellow_t_avg += yellow_t[i];
      }
      yellow_t_avg = yellow_t_avg/average_size;
      yellow_w_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        yellow_w_avg += yellow_w[i];
      }
      yellow_w_avg = yellow_w_avg/average_size;
      yellow_h_avg = 0;
      for (int i=0; i<average_size; i++)
      {
        yellow_h_avg += yellow_h[i];
      }
      yellow_h_avg = yellow_h_avg/average_size;
      //pros::lcd::print(2,"red box %3d,%3d,%3d,%3d", red_l_avg, red_t_avg, red_w_avg, red_h_avg);
    }
    yellow_area = yellow_w_avg*yellow_h_avg;
    yellow_center = yellow_l_avg+yellow_w_avg/2;
    yellow_bottom = yellow_t_avg+yellow_h_avg;
    // pros::lcd::print(2,"y_centre %d  y_bottom %d", yellow_center, yellow_bottom);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    pros::delay(10);
  }
}

// vision::signature RED_GOAL (1, 7763, 10029, 8896, -1821, -817, -1319, 1.800, 0);
// vision::signature YELLOW_GOAL (2, 1849, 2323, 2086, -3427, -2891, -3159, 1.400, 0);
// vision::signature BLUE_GOAL (3, 635, 1431, 1033, -4263, -3255, -3759, 2.000, 0);
// vex::vision vision1 ( vex::PORT1, 8, RED_GOAL, YELLOW_GOAL, BLUE_GOAL, SIG_4, SIG_5, SIG_6, SIG_7 );

// vision::signature BLUE_1 (1, -1209, -507, -858, -1311, -333, -822, 1.200, 0);
// vision::signature YELLOW_1 (2, 1257, 2093, 1675, -3837, -3303, -3570, 1.500, 0);
// vision::signature RED_1 (3, 6719, 8523, 7621, -1873, -1167, -1520, 3.000, 0);
// vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 3.000, 0);
// vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 3.000, 0);
// vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 3.000, 0);
// vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 3.000, 0);
// vex::vision vision1 ( vex::PORT1, 10, BLUE_1, YELLOW_1, RED_1, SIG_4, SIG_5, SIG_6, SIG_7 );


//RETAINED HOME DATA (BRIGHTNESS OFF)
// vision::signature RED_GOAL (1, 7763, 10029, 8896, -1821, -817, -1319, 1.800, 0);
// vision::signature YELLOW_GOAL (2, 1311, 3003, 2157, -3825, -3149, -3487, 1.600, 0);
// vision::signature BLUE_GOAL (3, -2575, -1615, -2095, 2795, 4645, 3720, 1.200, 0);
// vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 2.500, 0);
// vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 2.500, 0);
// vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 2.500, 0);
// vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 2.500, 0);
// vex::vision vision1 ( vex::PORT1, 50, RED_GOAL, YELLOW_GOAL, BLUE_GOAL, SIG_4, SIG_5, SIG_6, SIG_7 );

//FIRST SCHOOL DATA
// vision::signature RED_GOAL (1, 8453, 9681, 9067, -347, 1, -173, 3.800, 0);
// vision::signature YELLOW_GOAL (2, 2417, 2921, 2669, -3189, -2887, -3038, 3.000, 0);
// vision::signature BLUE_GOAL (3, -2169, -1489, -1829, 8831, 11021, 9926, 3.000, 0);
// vision::signature SIG_4 (4, 0, 0, 0, 0, 0, 0, 2.500, 0);
// vision::signature SIG_5 (5, 0, 0, 0, 0, 0, 0, 2.500, 0);
// vision::signature SIG_6 (6, 0, 0, 0, 0, 0, 0, 2.500, 0);
// vision::signature SIG_7 (7, 0, 0, 0, 0, 0, 0, 2.500, 0);
// vex::vision vision1 ( vex::PORT1, 35, RED_GOAL, YELLOW_GOAL, BLUE_GOAL, SIG_4, SIG_5, SIG_6, SIG_7 );

void vision_initialize()
{
  // vision_sensor.set_exposure(10);
  // blue_1 = vision_sensor.signature_from_utility(1, -1209, -507, -858, -1311, -333, -822, 1.200, 0);
  // yellow_1 = vision_sensor.signature_from_utility(2, 1257, 2093, 1675, -3837, -3303, -3570, 1.500, 0);
  // red_1 = vision_sensor.signature_from_utility(3, 6719, 8523, 7621, -1873, -1167, -1520, 3.000, 0);\
  vision_sensor.set_exposure(35);
  blue_1 = vision_sensor.signature_from_utility(3, -2169, -1489, -1829, 8831, 11021, 9926, 3.000, 0);
  yellow_1 = vision_sensor.signature_from_utility(2, 2417, 2921, 2669, -3189, -2887, -3038, 3.000, 0);
  red_1 = vision_sensor.signature_from_utility(1, 8453, 9681, 9067, -347, 1, -173, 3.800, 0);
  vision_sensor.set_signature(1, &blue_1);
  vision_sensor.set_signature(2, &yellow_1);
  vision_sensor.set_signature(3, &red_1);

  pros::Task vision_task (detect_mobile_goals, (void*)"PROSV5", TASK_PRIORITY_DEFAULT,
    TASK_STACK_DEPTH_DEFAULT, "Vision Task");
}
