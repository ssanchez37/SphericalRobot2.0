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
  WbDeviceTag beam = wb_robot_get_device("beam_motor");
  
  //WbDeviceTag longitudinal = wb_robot_get_device("long_motor");a
  //WbDeviceTag steer = wb_robot_get_device("steer_motor");
  
  double ts = wb_robot_get_basic_time_step()/1000;
  double rot;
  double i = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
  rot = 0.4*sin(i*ts); 
  wb_motor_set_position(beam, -0.1745);
  wb_motor_set_velocity(beam, 0.1);
  printf("rot = %frad \n", rot);
  //printf("phip[1] = %frad/seg \n", phip[1]);
  //printf("phip[2] = %frad/seg \n", phip[2]);
  i++;
  };
  


  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
