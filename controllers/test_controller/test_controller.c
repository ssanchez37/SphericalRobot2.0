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
#include <stdio.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gyro.h>
#include <webots/gps.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>

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
  wb_motor_set_position(longitudinal, 0.0);
  wb_motor_set_velocity(longitudinal, 0.0);
  wb_motor_enable_torque_feedback(longitudinal, TIME_STEP);
  wb_motor_set_position(steer, 0.0);
  wb_motor_set_velocity(steer, 0.0);
  
  double ts = wb_robot_get_basic_time_step()/1000;
  const double *phip, *pos, *theta, *thetap, *phishell;
  double P, I, D;
  int j = 1;
  double tiempo = 0;
  double x, y, z, xa, ya, za, x1, y1, z1, x1a, y1a, z1a;
  double th1, th2, th3, beta, beta2, phi, phi2, thsteer, theta2;
  double alphadeg, thsteerdeg, thetadeg, betadeg, beta2deg, phi2deg, phideg, th2deg, theta2deg, alpha;
  double thp1;
  double px, py, pz, px1, py1, pz1;
  double ppunto;
  double pmod, psqr, p1sqr, parg;
  double rc = 0;
  double Vd = -4;
  double sp = 0;
  double spz = 5;
  double spz2 = -5;
  double sp1 = 0;
  double sp2 = 5;
  double maxTorque = 0.68;
  double minTorque = -0.68;
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
  double ic = 0.536;
  double g = 9.81;
  thsteer = 0;
  alphadeg = 0;
  xa = 0;
  ya = 0;
  za = 0.198;
  x1a = 0;
  y1a = 0;
  z1a = 0.198;
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
  z = pos[1];
  y = pos[2];
  th1 = theta[1];
  th2 = pi+theta[0];
  thp1 = thetap[2];
  phi = beta - th1;
  phi2 = beta2 - th2;
  thetadeg = th1*180/pi;
  betadeg = beta*180/pi;
  beta2deg = beta2*180/pi;
  phideg = phi*180/pi;
  phi2deg = phi2*180/pi;
  th2deg = th2*180/pi;
  
  x1 = x;
  y1 = y;
  z1 = 0.198;
  
  // Cálculo del ángulo del plano
  
  px = x - xa;
  py = y - ya;
  pz = z - za;
  px1 = x1 - x1a;
  py1 = y1 - y1a;
  pz1 = z1 - z1a;
  ppunto = (px * px1) + (py * py1) + (pz * pz1);
  psqr = (px*px) + (py*py) + (pz*pz);
  p1sqr = (px1*px1) + (py1*py1) + (pz1*pz1);
  pmod = (sqrt(psqr)) * (sqrt(p1sqr));
  parg = ppunto/pmod;
  alpha = acos(parg);

  if (tiempo < 2){
    alphadeg = 0;
  }  
  else{
    if((pz >= 0)||((pz < 0)&&(phip[1] >= 0))){
      alphadeg = alpha*180/pi;
    }
    else if ((pz < 0)&&(phip[1] < 0)){
      alphadeg = -alpha*180/pi;
    }
  }
  
  
   // Control PID de Torque sintonizado utilizando señales estocássticas
  P = -43.7707018;  //Ganancia Proporcional
  I =  -2.9050182;  //Ganancia Integral
  D =  -0.2250206;  //Ganancia Derivativa
  
  error = Vd - phip[1];
  error_integral = (error_integral + error)*ts;
  error_derivative = (error - previous_error)/ts;
  if ((alphadeg < 1)&&(alphadeg >= -1)){//&&(phip[1] <= 0)){
    newTorque = P*error + I*error_integral + D*error_derivative;
    maxTorque = 0.1;
  }
  else if ((alphadeg >= 1)&&(alphadeg < 3)){
    newTorque = 0.2;
  }
  else if ((alphadeg >= 3)&&(alphadeg < 5)){
    newTorque = 0.4;
  }
  else if ((alphadeg >= 5)&&(alphadeg < 7)){
    newTorque = 0.6;
  }
  else if ((alphadeg >= 7)&&(alphadeg < 9)){
    newTorque = 0.8;
  }
  else if ((alphadeg >= 9)&&(alphadeg < 11)){
    newTorque = 1;
  }
  else if ((alphadeg >=11)&&(alphadeg < 13)){
    newTorque = 1.2;
  }
  else if ((alphadeg >= 13)&&(alphadeg < 15)){
    newTorque = 1.4;
  }
  else{
    newTorque = 0;
  }
  /*if ((alphadeg < 3)&&(alphadeg >= 0.2)&&(phip[1] < 0)){
    newTorque = 0.1;
  }
  else{
    newTorque = (alphadeg) * 0.095;
  }*/
  if (newTorque > maxTorque){
    newTorque = maxTorque;
  }
  else if (newTorque < minTorque){
    newTorque = minTorque;
  }
  /*if (newTorque > maxTorque){
    newTorque = maxTorque;
  }
  else if (newTorque < minTorque){
    newTorque = minTorque;
  }*/
  previous_error = error;
  
  xa = x;
  ya = y;
  za = z;
  x1a = x1;
  y1a = y1;
  z1a = z1;
  
  tiempo = j*0.064;
  //if (x < 0){
    wb_motor_set_torque(longitudinal, newTorque);
    wb_motor_set_position(steer, 0.0);
    wb_motor_set_velocity(steer, 1.5);
    wb_motor_set_control_pid(steer, 1.7, 1.74, 0.01);
  //}
  /* else if (x >= 0){
    wb_motor_set_torque(longitudinal, newTorque);
    rc = (sp-spz)/2;
    thsteer = (((m1+m2)*r1+m2*(r1-r2))*(Vd*Vd*r1*r1)+m2*g*r2*r1)/(m2*g*r2*rc);
    //double thsteer = ((r1*phip[0]*phip[0])*(ic-(m2*r1*r2)+(r1*r1)*(m1+m2)))/(m2*g*r2*rc);
    thsteer = (1 - 0.01)*(thsteer - 0.032*thetap[2])+0.01*th2;
    wb_motor_set_position(steer, thsteer);
    wb_motor_set_velocity(steer, 1.5);
    wb_motor_set_control_pid(steer, 1.7, 1.74, 0.01);
    //wb_motor_set_control_pid(steer, 0.5, 0.5, 0.1);
  }*/
   j++;
  
  /*wb_motor_set_torque(longitudinal, newTorque);
  rc = (sp2-sp1)/2;
  thsteer = ((r1*phip[0]*phip[0])*(i1+m2*r1*r2+((r1*r1)*(m1+m2))))/(m2*g*r2*rc);
  //thsteer = ((r1*phip[0]*phip[0])*(i1 - m2*r1*r2 + (r1*r1*(m1+m2))))/(m2*g*r2*rc);
  //thsteer = (((m1+m2)*r1+m2*(r1-r2))*(phip[0]*phip[0]*r1*r1)+m2*g*r2*r1)/(m2*g*r2*rc);
  thsteer = (1 - 0.01)*(thsteer - 0.032*thp1)+0.01*th2;
  theta2 = beta - phi;
  wb_motor_set_position(steer, 0.0);
  wb_motor_set_velocity(steer, 0.0);
  wb_motor_set_control_pid(steer, 2.3, 4, 0); */
 
  printf("Theta1 = %f°\n", thetadeg);
  printf("Theta2 = %f°\n", th2deg);
  printf("Beta = %f°\n", betadeg);
  printf("Phi = %f°\n", phideg);
  printf("X = %fm\n", x);
  printf("Y = %fm\n", y);
  printf("Z = %fm\n", z);
  printf("X1 = %fm\n", px1);
  printf("Y1 = %fm\n", py1);
  printf("Z1 = %fm\n", pz1);
  printf("W = %frad/s\n", thp1);
  printf("Wsph = %frad/s\n", phip[1]);
  printf("Debug = %frad\n", newTorque);
  printf("Debug2 = %f\n", tiempo);
  printf("Debug3 = %f°\n", alphadeg);
  maxTorque = 0.68;
  minTorque = -maxTorque;
  
  };
  
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
