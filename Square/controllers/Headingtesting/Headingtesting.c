/*
 * File:          pid_square.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.

Time step given as 16 (ms I think). default acceleration is A = 2 rad/s² for the motors (scene treet) */ 
#include <math.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include<webots/inertial_unit.h>


#define TIME_STEP 16
#define MAX_SPEED 10.0 
#define DISTANCE_TOLERANCE 1.5
#define WHEEL_RADIUS 0.4
#define ts 0.016


WbDeviceTag fl_motor, fr_motor, bl_motor, br_motor;
WbDeviceTag imu; 
WbDeviceTag ds[8];

//misc. variables 
double prev_heading_error, differential_error, integral_error;
double P = 0.03, D = 0.01, I = 0.01, offset = -90.0, base_speed = 3; //[rad/s] base_speed is also PID limit  
double heading_t = -90.0, pid_out_limit = 90.0, Vprevious_step, Vdesired, Vc; 
double right_speed, left_speed;
bool   right_obstacle, left_obstacle = false; 
/*
 *
 */
 
//given delay function 
void delay(int milliseconds) { 
    // Storing start time 
    double start_time = wb_robot_get_time()*1000; 
    // looping till required time is not achieved 
    while (wb_robot_get_time()*1000 < start_time + milliseconds){
    wb_robot_step(TIME_STEP); 
    }
} // delay 
 
 /* Stopping function */
void stop() {
  double rds =0 ; 
  wb_motor_set_velocity(fl_motor, rds);
  wb_motor_set_velocity(fr_motor, rds);
  wb_motor_set_velocity(br_motor, rds);
  wb_motor_set_velocity(bl_motor, rds);
  Vdesired =  rds;
} //stop

/*PID control to control the heading of a wheeled robot */
double PID_Controller() {
         double heading_c = wb_inertial_unit_get_roll_pitch_yaw(imu)[2]*180/M_PI; //[degrees] c for current
                  if (heading_c < -179.0) 
                  {   heading_t = -1* heading_t; }    
                  if (heading_c > 179.0) {
                      heading_t = -1* heading_t;
                  }
         double error = heading_t - heading_c; 
             error =  (error < -359) ?  error + 359 : error;
             error =  (error > +359) ?  error - 359 : error;  
    
       //  printf("error(heading): %f degrees\n", error);
         differential_error = (prev_heading_error - error) / ts;
         integral_error += error * ts;
         Vc = P * error + I * integral_error + D * differential_error ; //[rad/s] I'm guessing is the current vel
         Vdesired = base_speed; //desired velocity set with the set with the wb_motor_set_velocity function
         
         if (fabs(Vc) > Vdesired) 
             Vc = (Vc/fabs(Vc)) * Vdesired;    //  Vc = sign(Vc) * Vdesired;        
         double  A = (fabs(wb_motor_get_acceleration(fl_motor)) + fabs(wb_motor_get_acceleration(fr_motor)))/2; 
          double a;
         if (A != -1){
            a = (Vc - Vprevious_step) / ts;//a is the acceleration required to reach Vc in one time step. Vprevious_steprevious_step is the motor velocity of the previous time step
            if (fabs(a) > A)
                a = (a < 0) ? -A: A;  
          Vc = Vprevious_step + a * ts; }
          /*  error_integral and previous_error are both reset to 0 after every sign change of the PID */   
          integral_error = (error*prev_heading_error <= 0) ? 0: integral_error; 
          prev_heading_error =  error;
           
          Vc = (fabs(Vc) > MAX_SPEED) ? (Vc/fabs(Vc) * base_speed): Vc; 
          
          if (right_obstacle || left_obstacle) { 
                Vc = 0.3* MAX_SPEED; }
          
          Vprevious_step =  Vc; //or should it be Vdesired 
          printf("PID Vc %f rad/s, heading_c %f °, heading_t %f °, error %f °\n", Vc, heading_c, heading_t, error);
          return Vc;
          
       
} 

/*/double PID_Controller4Distance() {
       double distance_c [4]; 
       for (k = 0; k<4; k++) {
            distance[k] =  (0.7611*pow(wb_distance_sensor_get_value(ds[k]),-0.9313)) -0.1252 ; // [m]
   //  printf("distance value %g m\n, PID: %f \n", ds_values[k], pid_output);
       } 


} */

/*set_speed in radians per sec */
void set_Speed(double left_rds,double right_rds) {
     left_rds = (left_rds > MAX_SPEED) ? 0.3*MAX_SPEED: left_rds; 
     right_rds = (right_rds > MAX_SPEED) ? 0.3*MAX_SPEED: right_rds; 
   /* set motor rotation speed. set_Speed just applies the "disired" velocity. comppuations w.r.t
      base speed need to be done prior to calling function. simplistic */
  wb_motor_set_velocity(fl_motor, left_rds);wb_motor_set_velocity(fr_motor, right_rds);
  wb_motor_set_velocity(bl_motor, left_rds);wb_motor_set_velocity(br_motor, right_rds);
  
   // double rds = (fabs(left_rds)< fabs(right_rds)) ? right_rds: left_rds; //display largest speed 
 // double kmh = rds * 1000.0 / 3600.0 / WHEEL_RADIUS;
 // printf("left speed: %g km/h, right speed: %g km/h\n", left_rds * 1000.0 / 3600.0 / WHEEL_RADIUS, right_rds* 1000.0 / 3600.0 / WHEEL_RADIUS);

  Vdesired = (fabs(wb_motor_get_velocity(fl_motor)) + fabs(wb_motor_get_velocity(fr_motor))) / 2;//Need to address this. what should Vdesired be after setting speed after turns
} //set_Speed
 
/* Main. remeber timestep is 16ms, and the refresh rate for the motors and imu is 32ms  */
int main(int argc, char **argv) {
  wb_robot_init();
  // find wheels
  fl_motor = wb_robot_get_device("FLMotor"); fr_motor = wb_robot_get_device("FRMotor"); 
  bl_motor = wb_robot_get_device("BLMotor"); br_motor = wb_robot_get_device("BRMotor");
  // find imu
  imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, TIME_STEP);
  // intialise motors
  wb_motor_set_position( fl_motor, INFINITY); wb_motor_set_position( fr_motor, INFINITY); 
  wb_motor_set_position( bl_motor, INFINITY); wb_motor_set_position( br_motor, INFINITY); 
  // distance/proximity sensors 
  int k =0;
 /* char ds_names[4][15] = {"ds_frontright", "ds_rightside","ds_frontleft","ds_leftside"}; 
  for (int k=0; k<4; k++) { 
    ds[k] =  wb_robot_get_device(ds_names[k]); //note to self: ds is the distance sensor tags, not the distances
    wb_distance_sensor_enable(ds[k], TIME_STEP); */
    //}           
   // initiate movement   
   left_speed = right_speed = Vprevious_step =  base_speed; 
   set_Speed(left_speed,right_speed); 
   bool squared =  false; 
    

/* feedback loop */
while (wb_robot_step(TIME_STEP) != -1) {
      //set_Speed(left_speed,right_speed); 
      double PID = PID_Controller(); 
       
      //Vdesired = (wb_motor_get_velocity(fl_motor) + wb_motor_get_velocity(fr_motor)) / 2 ; 
      //read ds sensors
     /* double distance[4]; 
       for (k = 0; k<4; k++) {
            distance[k] =  (0.7611*pow(wb_distance_sensor_get_value(ds[k]),-0.9313)) -0.1252 ; // [m] */
   //  printf("distance value %g m\n, PID: %f \n", ds_values[k], pid_output);
       //}    
      // detect obsctacles
     // right_obstacle = distance[0] < 1.3 ;  
      //left_obstacle = distance[2] < 1.3;
    /*  
      if (right_obstacle && left_obstacle){ 
          left_obstacle = (distance[0]< distance[2]) ? false:true;
          left_obstacle = (distance[0] == distance[2]) ? false: true; }
    // modify speeds according to obstacles
    if (left_obstacle) {
      // turn right <----
      printf("turning right "); 
      heading_t = 90.0; //make it think it's off course by +90 degrees so it can "correct" 90 degrees right (cw, whihc is -90)
      PID =  PID_Controller(); 
      
      left_speed = 0;
      right_speed = -(fabs(PID));
          
      squared = true;
     } else if (right_obstacle) {
      // turn left ---->
      
      printf("turning left");
      heading_t = -90.0; 
      PID =  PID_Controller(); 
      
      
      left_speed = -(fabs(PID));
      right_speed = 0;
          
      squared = true;  
    } 
  
     // write actuators inputs
     if (squared){
        heading_t = 0.0;} */
    bool withinrange = (PID > -0.1 && PID < 0.1); 
    if (withinrange == false  && squared == false){   
      if (PID < 0) {
      //right turn ---->: right_speed = - left_speed (turn right wheel backwards)
      left_speed = (base_speed + fabs(PID)); right_speed = (base_speed - fabs(PID)); //negative
        
      
      } else if (PID > 0) { //errerd to the right actually since yaw is posiyive clockwise. Yes.
      //left turn <----: left_speed = -right_speed
      left_speed = (base_speed - fabs(PID)); right_speed = (base_speed + fabs(PID));
           
      } else if (PID == 0) {
      left_speed = right_speed =  base_speed;
      }
       
      withinrange = true; 
      set_Speed(left_speed, right_speed); 
    } //if withinrange
    
    //actuation after correction (drive straight)
    squared = false; 
    
    
    
 
    
}; //while

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
