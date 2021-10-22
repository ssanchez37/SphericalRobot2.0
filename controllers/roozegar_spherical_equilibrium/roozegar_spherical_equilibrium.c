/*
 * File:          test_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  WbDeviceTag longitudinal = wb_robot_get_device("long_motor");
  
  double ts = wb_robot_get_basic_time_step()/1000;
  double alpha = 0;
  double the;
  double i = 0;
  double rho = 0.18;
  double rG = 0.101;
  double tmp1;
  double tmp2;

  while (wb_robot_step(TIME_STEP) != -1) {
  alpha = -0.1745;//0.4*sin(i*ts);
  tmp1 = sin(alpha);
  tmp2 = rho/rG*tmp1;
  the = asin(tmp2) - alpha;
  
  wb_motor_set_position(longitudinal, the);
  wb_motor_set_velocity(longitudinal, 1.5);
  
  printf("the = %frad \n", the);
  printf("alpha = %frad \n", alpha);
  //printf("phip[2] = %frad/seg \n", phip[2]);
  i++;
  };
  


  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
