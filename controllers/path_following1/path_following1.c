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
  WbDeviceTag steer_pos = wb_robot_get_device("steer_pos");
  WbDeviceTag imu_shell = wb_robot_get_device("imu_shell");
  wb_gyro_enable(gyroshell, TIME_STEP);
  wb_inertial_unit_enable(imu_shell, TIME_STEP);
  wb_gyro_enable(gyro1, TIME_STEP);
  wb_gps_enable(my_gps, TIME_STEP);
  wb_position_sensor_enable(long_pos, TIME_STEP);
  wb_position_sensor_enable(steer_pos, TIME_STEP);
  wb_inertial_unit_enable(inertial, TIME_STEP);
  
  double ts = wb_robot_get_basic_time_step()/1000;
  const double *phip, *pos, *theta, *thetap, *phishell;
  double P, I, D;
  double x, y, z;
  int i ;
  int j = 0;
  double pcoord, psqr, qsqr, pmod, cospq, dseta, thetapdes, phides;
  double temp, th1, th2, th3, beta, beta2, phi, phi2, thsteer, theta2, rc;
  double thsteerdeg, thetadeg, betadeg, beta2deg, phi2deg, phideg, th2deg, theta2deg;
  double thp1;
  double Vd = -4;
  double xsp, ysp, zsp;
  double xpsp, ypsp, zpsp;
  double sp0[] ={0, 0, 0};
  double sp1[] = {0, 5, 0};
  double x0[] = {0, 0, 0};
  double kphi = 4;
  double kth = 4;
  double ke = 0.5;
  double Ppsquare, Ppnorma, errorsquare, errornorma;
  //double Pact[] = {0, 0, 0};
  //double Psp[] = {0, 0, 0};
  double hgorro[] = {0, 0, 0};
  double lenght = 3;
  double maxTorque = 0.1;
  double minTorque = 0.0;
  double error[] = {0, 0, 0};
  double error_integral = 0;
  double error_derivative = 0;
  double previous_error = 0;
  double newTorque = 0;
  double pi = 3.1416;
  double r1 = 0.2;
  double r2 = 0.15;
  double m1 = 0.5;
  double m2 = 0.639;
  double i1 = 0.0133333;
  double g = 9.81;
  double phisp[]={0,0,0};
  //double rG = m2*r2/(m1+m2);
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
  phip = wb_gyro_get_values(gyroshell);
  thetap = wb_gyro_get_values(gyro1);
  pos = wb_gps_get_values(my_gps);
  beta = wb_position_sensor_get_value(long_pos);
  phishell = wb_inertial_unit_get_roll_pitch_yaw(imu_shell);
  beta2 = wb_position_sensor_get_value(steer_pos);
  theta = wb_inertial_unit_get_roll_pitch_yaw(inertial);

//Declaración de variables obtenidas de los sensores
  x = pos[0];
  z = pos[1] - 0.2;
  y = pos[2];
  // z = 0;
  double Pact[] = {x, y, z};
  th1 = theta[0];
  th2 = theta[1];
  th3 = theta[3];
  thp1 = thetap[0];
  phi = beta - th1;
  phi2 = beta2 - th2;
  thetadeg = th1*180/pi;
  betadeg = beta*180/pi;
  beta2deg = beta2*180/pi;
  phideg = phi*180/pi;
  phi2deg = phi2*180/pi;
  th2deg = th2*180/pi;
  
 /* int i;
  
  for (i=0; i < lenght; i++) {
    phisp[i] = sp1[i]/r1;
  }
  rc = (sp1[1]-sp0[1])/2;
   
  thsteer = atan2(r1, rc);
  thsteer = (1 - 0.01)*(thsteer - 0.032*thp1)+0.01*th2;
 wb_motor_set_control_pid(longitudinal, 1.0, 0.0, 0.0);
 wb_motor_set_position(longitudinal, INFINITY);
 //wb_motor_set_position(longitudinal, phisp[0]);
 wb_motor_set_velocity(longitudinal, 1.5);
 wb_motor_set_control_pid(steer, 1.0, 0.0, 0.0);
 wb_motor_set_position(steer, thsteer);
 wb_motor_set_velocity(steer, 1.5);
  */
  
  xsp = 2*cos(0.05*j*ts);
  ysp = 2*sin(0.05*j*ts);
  // zsp = 0;
  zsp = -0.2*(cos(2*xsp)+cos(2*ysp)-2);
  xpsp = -0.1*sin(0.05*j*ts);
  ypsp = 0.1*cos(0.05*j*ts);
 // zpsp = 0;
 zpsp = -0.04*sin(0.05*j*ts)*sin(4*cos(0.05*j*ts)) + 0.04*cos(0.05*j*ts)*sin(4*sin(0.05*j*ts));
 double Psp[] = {xsp, ysp, zsp};
 double Ppsp[] = {xpsp, ypsp, zpsp};
  
  for (i = 0; i < lenght; i++){
    error[i] = Psp[i] - Pact[i];
    hgorro[i] = Pact[i] - x0[i];
  }
  pcoord = error[0]*hgorro[0] + error[1]*hgorro[1] + error[2]*hgorro[2];
  psqr = pow(error[0], 2) + pow(error[1], 2) + pow(error[2], 2); // error[0]*error[0] + error[1]*error[1] + error[2]*error[2];
  qsqr = pow(hgorro[0], 2) + pow(hgorro[1], 2) + pow(hgorro[2], 2); //hgorro[0]*hgorro[0] + hgorro[1]*hgorro[1] + hgorro[2]*hgorro[2];
  pmod = (sqrt(psqr))*(sqrt(qsqr));
  cospq = pcoord/pmod; 
  dseta =  acos(cospq);
  Ppsquare = pow(Ppsp[0], 2) + pow(Ppsp[1], 2) + pow(Ppsp[2], 2); //Ppsp[0]*Ppsp[0] + Ppsp[1]*Ppsp[1] + Ppsp[2]*Ppsp[2];
  Ppnorma = sqrt(Ppsquare);
  //errorsquare = error[0]*error[0] + error[1]*error[1] + error[2]*error[2];
  errornorma = sqrt(psqr);
  thetapdes = fabs((Ppnorma/r1) + (kth*errornorma*cos(dseta))/(ke+errornorma)); 
  phides = kphi*dseta;
  
 /* if (phides > 0.5236) {
    phides = 0.5236;
  }
  else if (phides < -0.5236){
    phides = -0.5236;
  }*/
  
  x0[0] = Pact[0];
  x0[1] = Pact[1];
  x0[2] = Pact[2];
  
  j = j + 2;
  
  wb_motor_set_velocity(longitudinal, thetapdes);
  wb_motor_set_position(longitudinal, INFINITY);
  wb_motor_set_position(steer, phides);
  wb_motor_set_velocity(steer, 1);
  
  printf("Theta1 = %f°\n", thetadeg);
  printf("Theta2 = %f°\n", th2deg);
  printf("Beta = %f°\n", betadeg);
  printf("Phi = %f°\n", phideg);
  printf("X = %fm\n", x);
  printf("Y = %fm\n", y);
  printf("W = %frad/s\n", thp1);
  printf("Wsph = %frad/s\n", phip[0]);
  printf("Debug = %frad\n", x);
  printf("Debug2 = %frad\n", y);
  printf("Debug3 = %frad\n", z);
  };
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
