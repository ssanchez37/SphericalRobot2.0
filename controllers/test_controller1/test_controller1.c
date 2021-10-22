/*
 * File:          test_controller1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include<webots/motor.h>
#include<webots/gyro.h>
#include <stdio.h>

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
  WbDeviceTag gyroshell = wb_robot_get_device("gyroshell");
  wb_gyro_enable(gyroshell, TIME_STEP);
  double ts = wb_robot_get_basic_time_step()/1000;
  double P, I, D;
  double Vd = -4;
  double maxTorque = 0.3;
  double minTorque = -0.3;
  double error = 0;
  double error_integral = 0;
  double error_derivative = 0;
  double previous_error = 0;
  double newTorque = 0;
  const double *phip;
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
    
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
       // Control PID de Torque sintonizado utilizando señales estocássticas
  P = -43.7707018;  //Ganancia Proporcional
  I =  -2.9050182;  //Ganancia Integral
  D =  -0.2250206;  //Ganancia Derivativa
  phip = wb_gyro_get_values(gyroshell);
  error = Vd - phip[0];
  error_integral = (error_integral + error)*ts;
  error_derivative = (previous_error - error)/ts;
  newTorque = P*error + I*error_integral + D*error_derivative;
  if (newTorque > maxTorque){
    newTorque = maxTorque;
  }
  else if (newTorque < minTorque){
    newTorque = minTorque;
  }
  previous_error = error;
     wb_motor_set_torque(longitudinal, newTorque);
  };

  /* Enter your cleanup code here */
 printf("Wsph = %frad/s\n", phip[0]);
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
