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
#include<webots/gps.h>
#include<webots/position_sensor.h>
#include<webots/inertial_unit.h>

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
  WbDeviceTag steer = wb_robot_get_device("steer_motor");
  WbDeviceTag my_gps = wb_robot_get_device("my_gps");
  WbDeviceTag gyroshell = wb_robot_get_device("gyroshell");
  WbDeviceTag gyro1 = wb_robot_get_device("gyro1");
  WbDeviceTag long_pos = wb_robot_get_device("long_pos");
  WbDeviceTag inertial = wb_robot_get_device("inertial");
  wb_gyro_enable(gyroshell, TIME_STEP);
  wb_gyro_enable(gyro1, TIME_STEP);
  wb_gps_enable(my_gps, TIME_STEP);
  wb_position_sensor_enable(long_pos, TIME_STEP);
  wb_inertial_unit_enable(inertial, TIME_STEP);
  
  double ts = wb_robot_get_basic_time_step()/1000;
  const double *phip, *pos, *theta, *thetap;
  double P, I, D;
  double x, y;
  double th1, th2, beta, phi, thsteer, rc;
  double thetadeg, betadeg, phideg, th2deg;
  double thp1;
  double Vd = -4;
  double sp1 = 0;
  double sp2 = 5;
  double maxTorque = 0.1;
  double minTorque = -0.1;
  double error = 0;
  double error_integral = 0;
  double error_derivative = 0;
  double previous_error = 0;
  double newTorque = 0;
  double pi = 3.1416;
  double r1 = 0.2;
  double r2 = 0.18;
  double m1 = 0.5;
  double m2 = 0.639;
  double i1 = 0.01333;
  double g = 9.81;
  //double rG = m2*r2/(m1+m2);
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
  phip = wb_gyro_get_values(gyroshell);
  thetap = wb_gyro_get_values(gyro1);
  pos = wb_gps_get_values(my_gps);
  beta = wb_position_sensor_get_value(long_pos);
  theta = wb_inertial_unit_get_roll_pitch_yaw(inertial);

//Declaración de variables obtenidas de los sensores
  x = pos[0];
 // z = pos[1];
  y = pos[2];
  th1 = theta[0];
  th2 = theta[1];
  thp1 = thetap[0];
  phi = beta - th1;
  thetadeg = th1*180/pi;
  betadeg = beta*180/pi;
  phideg = phi*180/pi;
  th2deg = th2*180/pi;
  
   // Control PID de Torque sintonizado utilizando señales estocássticas
  P = -43.7707018;  //Ganancia Proporcional
  I =  -2.9050182;  //Ganancia Integral
  D =  -0.2250206;  //Ganancia Derivativa
  
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
  rc = (sp2-sp1)/2;
  thsteer = ((r1*Vd*Vd)*(i1 - m2*r1*r2 + (r1*r1*(m1+m2))))/(m2*g*r2*rc);
  //thsteer = (((m1+m2)*r1+m2*(r1-r2))*(Vd*Vd*r1*r1)+m2*g*r2*r1)/(m2*g*r2*rc);
 // thsteer = (1 - 0.01)*(thsteer - 0.032*thp1)+0.01*th2;
  wb_motor_set_position(steer, thsteer);
  wb_motor_set_velocity(steer, 1.5);
  wb_motor_set_control_pid(steer, 100, 0, 0); 
  
  printf("Theta1 = %f°\n", thetadeg);
  printf("Theta2 = %f°\n", th2deg);
  printf("Beta = %f°\n", betadeg);
  printf("Phi = %f°\n", phideg);
  printf("X = %fm\n", x);
  printf("Y = %fm\n", y);
  printf("W = %frad/s\n", thp1);
  printf("Wsph = %frad/s\n", phip[0]);
  printf("Debug = %frad\n", thsteer);
  };
  


  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
