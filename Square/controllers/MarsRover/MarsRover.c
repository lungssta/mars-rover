/*  Assigment Group 37
 * File:          MarsRover.c
 * Date:
 * Description: MECN3012Project
 * Author: Lunga Mkhwanazi, Emmanuel Netshidzivhe, Chumani Mayosi
 * Modifications:
 */

/* Initial pos for the beacons at robot pos X  = 25, Z =25, bearing = 0





*/




/*Time step given as 16 (ms I think). default acceleration is A = 2 rad/s² for the motors (scene treet) */ 
#include <math.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <stdbool.h>
#include <time.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <webots/supervisor.h>
#include <webots/inertial_unit.h>
#include <webots/lidar.h>
//definitions
#define TIME_STEP 16
#define MAX_SPEED 10.0 
#define DISTANCE_TOLERANCE 1.5
#define WHEEL_RADIUS 0.4
#define ts 0.016
#define PI 3.141593

//devices
WbDeviceTag fl_motor, fr_motor, bl_motor, br_motor;

//hokuyo lidar. resolution set to 50 in the scene tree
WbDeviceTag hokuyo; 
double hokuyo_range = -1.0, hokuyo_fov = -1.0; int hokuyo_size = -1;
bool Assigned = false; 
double heading_prev; 
int four ; 

//inertial measurement unit 
WbDeviceTag imu; 
double pitch_f, pitch_i, heading, pitch_prev, delta; 
/*Never shall we target 180.0. Instead we will refer to this heading as zero */


//Manipulator 
 WbDeviceTag Motor1, Motor2, Motor3, Motor4, Motor5, Motor6; 

//Speed variables & heading PID control 
double prev_heading_error, differential_error, integral_error;
double heading_c;
double P = 0.03, D = 0.01, I = 0.01, offset = -90.0, base_speed = 5.2; //[rad/s] base_speed is also PID limit  
double heading_t = -0.0, pid_out_limit = 90.0, Vprevious_step, Vdesired, Vc; 
double right_speed, left_speed;
bool   right_obstacle, left_obstacle = false; 



/* Custom functions 
===========================================================================================================================================*/ 
//given delay function 
void delay(int milliseconds) { 
    double start_time = wb_robot_get_time()*1000; 
      while (wb_robot_get_time()*1000 < start_time + milliseconds){
          wb_robot_step(TIME_STEP); // looping till required time is not achieved 
    }
} /* delay 
 
Stopping function */
void stop() {
  double rds =0 ; 
  wb_motor_set_velocity(fl_motor, rds);
  wb_motor_set_velocity(fr_motor, rds);
  wb_motor_set_velocity(br_motor, rds);
  wb_motor_set_velocity(bl_motor, rds);
  Vdesired =  rds;
} /*stop

PID control to control the heading of a wheeled robot */
double PID_Controller() {
          heading_c = wb_inertial_unit_get_roll_pitch_yaw(imu)[2]*180/M_PI; //[degrees] c for current
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
                Vc = 0.52* MAX_SPEED; }
          
          Vprevious_step =  Vc; //or should it be Vdesired 
          printf("PID Vc %f rad/s, heading_c %f °, heading_t %f °, error %f °\n", Vc, heading_c, heading_t, error);
          return Vc;    
} /*PID_Controller

find first corner/ first bearing, get starting pitch, and heading */
int FirstCorner(int delaytime){
     delay(TIME_STEP);
    // double pitch_i = wb_inertial_unit_get_roll_pitch_yaw(imu)[1] * 180/M_PI; 
     delay(delaytime);//keep driving around to get a sense of the bearings
     double pitch_f = wb_inertial_unit_get_roll_pitch_yaw(imu)[1] * 180/M_PI; 
    
    //wite a code that assumes a initial pitch about zero or near 
    // double delta = fabs(pitch_f) - fabs(pitch_i); 
     int pitch_sign; 
          if (pitch_f == 0){ 
            pitch_sign = 0;
        } else {
            pitch_sign = pitch_f/fabs(pitch_f);
        }
     
      /* if (pitch_f < 0 && fabs(delta) < 5.0) { //[deg] 
            pitch_sign = 0; }  */
    /* * driving near centre of the map at some acute unevenness (so it appears as climbing),& would actually reach centre-edge if let on to drive
       * this implies current is heading either towards -pi/2 or pi/2*/ 
     heading = wb_inertial_unit_get_roll_pitch_yaw(imu)[2]*180/M_PI;         
     
     return pitch_sign; 
   
} /* 1stCorner */



/* * this code needs to be initiated precisely after the rood has entered a climb, 
  * i.e pitch is negative and climbing
  
   return the condition/state: 
    * -1 == climbing
    *  0 == stationary /horizontal
    *  1 == descending   */
int Highest(){
 //read sensors (&lidar)
     pitch_f = wb_inertial_unit_get_roll_pitch_yaw(imu)[1] * 180/M_PI;
     double delta = (fabs(pitch_f) -  (pitch_i)) * 180/M_PI;
    /* ixels are stored in scan lines running from left to right and from first to last layer.  
    */
     const float *range_image = wb_lidar_get_range_image(hokuyo); /* array with a Number of elements specified by the resolution */
    
    /* *Resolution of 51 
       *Band of rays that workk between 90 and 180 are rays [17-34] */
      int highestlocation = 17; int second_highestlocation = 18; 
      for (int k = 17 ; k < 35; k++) {
        // printf(" %.2f m  fov heading %.2f °,  index number %d \n",  *(range_image+k) ,((wb_lidar_get_fov(hokuyo)/ hokuyo_size) * k) * 180/M_PI, k); 
          if (range_image[k] < range_image[highestlocation]) {
               second_highestlocation =  highestlocation; 
               highestlocation = k;
             printf("highest: %.2f m,  fov heading %.2f °,  index number %d \n",  *(range_image+k) ,((hokuyo_fov/ hokuyo_size) * k) * 180/M_PI, k);
             //  printf(" %.2f m,  fov heading %.2f °,  index number %d \n",  *(range_image+second_highestlocation) ,((wb_lidar_get_fov(hokuyo)/ hokuyo_size) * second_highestlocation) * 180/M_PI, second_highestlocation);
           }
        } 
     
    double heading_highestlocation = ((hokuyo_fov/ hokuyo_size) * highestlocation) * 180/M_PI;
    double heading_second_highestlocation = ((hokuyo_fov/ hokuyo_size) * second_highestlocation) * 180/M_PI;
  //  double shift_137 =  ((wb_lidar_get_fov(hokuyo)/ hokuyo_size) * 26) * 180/M_PI; // ==137 
    double shift_137 = 135.0; 
    /* convert lidar heading to Worldinfo/robot heaading 
    
    heading_highestlocation = fabs(heading_highestlocation) - fabs(heading); 
    heading_highestlocation = heading/fabs(heading) * heading_highestlocation;
    */
    heading = wb_inertial_unit_get_roll_pitch_yaw(imu)[2] * 180/M_PI; 
    double alpha = fabs(heading_highestlocation) - fabs(heading);
        
     if (heading < 0  && heading >= -180.0) {//negative angles inc by becoming more negative  
             
        
         heading_highestlocation = -1 * (heading_highestlocation - shift_137 + alpha)  ; 
         
         
         
         //heading_highestlocation = (fabs(heading_highestlocation) > fabs(heading)) ? heading_highestlocation - alpha: heading_highestlocation + alpha; 
         
         heading_second_highestlocation = -1 * (heading_second_highestlocation - shift_137 + alpha) ;
         
         
         
     }
      else if (heading > 0 && heading <= 180.0) { //positive angles 
         heading_highestlocation =   (heading_highestlocation);
         
         heading_second_highestlocation =  (heading_second_highestlocation );
     }
     else if (heading == 0) { 
       heading_highestlocation = heading_highestlocation - shift_137;
        if (fabs(heading_highestlocation) > 180){
            heading_highestlocation = heading_highestlocation - 180; 
        }
        
     heading_second_highestlocation =  shift_137 - heading_second_highestlocation+ alpha; 
    }
    printf("heading highest location::  %.2f °\n", heading_highestlocation); 
  
    heading_t =  heading_highestlocation;   /* so long as robot is climbing, move towards the highest location */ 
    int condition =  -1; //default value is climb. robot should tell itself when to stop climbing based on heading and pitch. 

    if (delta >= 14.0 ) {
       //should be face a high point. heading is good. it's all in lidar's hands now. 
       condition = pitch_f >= 0 ? 0:-1; 
    } 
    else if (delta < 14.0 && pitch_f >= 0) {
/*   * delta criteria unmet and robot changed direction. 
     * still needs to climb to the highest point
     * re-ignite search for highest point       */
        if ( (fabs(heading) - fabs(heading_prev)) > 8.0){
       
            heading_t = heading_prev;}
         else {
            heading_t = heading_second_highestlocation; 
          }
      condition =  -1; 
    } else if (delta < 14.0 && pitch_f < 0 ) {
      /* delta criteria unmet but robot is still climbing 
       * let it continue  */  
      condition = -1; 
      heading_t =  heading_highestlocation; 
     } else if (delta > 14.0 && pitch_f > 1.5) {
          /* delta criteria met but robot is moving down having not stopped at the hill
           */ 
           heading_t = -1 * heading_highestlocation; 
           condition = -1;
     } else if (delta < 14 && pitch_f >= 0) {
            /* delta criteria met but robot is at the hill top 
           */ 
           if (pitch_i <=0)
               condition = -1; 
     
     } else if (delta > 14 && (pitch_i < 0 || pitch_f < 0) ) {
         condition = -1; 
     
     } 
     
     if (pitch_i < 0) {
       condition = pitch_f >= 0 ? 0:-1; 
     
     }
     
     /* pitch = -0.153 =  getting on the hill top
      * pitch = -0.018 = 1 ° getting stable on hill
      * pitch = 0.0008 basically horz on the hill
     
     */
      
      /*  high point locations given 
          y1 == 5.01 m  . .  at this point( lowest peak) the slant was about 14 -16 °, so we used 14 as the min angle  that needs to be reached by robot
          y2 == 9.10 m 
          y3 == 9.16 m
          y4 == 5.22 m 
    */
    heading_prev =  heading_t; pitch_prev =pitch_f;
    printf("f-End. heading target in Highest()  %.2f °\n", heading_t);
 
    return condition; 
} /*  Highest()
      *function should run so long as robot is climbing 
      *It'll also tell the robot to stop once it's reached the highest point
*/


/*set_Speed in radians per sec */
void set_Speed(double left_rds,double right_rds) {
     left_rds = (left_rds > MAX_SPEED) ? left_rds/fabs(left_rds)*0.52*MAX_SPEED: left_rds; 
     right_rds = (right_rds > MAX_SPEED) ? right_rds/fabs(right_rds)*0.52*MAX_SPEED: right_rds; 
/*    set motor rotation speed. set_Speed just applies the "disired" velocity. comppuations w.r.t
      base speed need to be done prior to calling function. simplistic */
     wb_motor_set_velocity(fl_motor, left_rds);wb_motor_set_velocity(fr_motor, right_rds);
     wb_motor_set_velocity(bl_motor, left_rds);wb_motor_set_velocity(br_motor, right_rds);
  
     printf("left speed: %g km/h, right speed: %g km/h\n", left_rds * 1000.0 / 3600.0 / WHEEL_RADIUS, right_rds* 1000.0 / 3600.0 / WHEEL_RADIUS);

  //Need to address this.: what should Vdesired be after setting speed after turns. atm, working theory is Vdesired = base_speed
} /* set_Speed s  */

void Park(int corner ) {

base_speed = left_speed = right_speed = Vprevious_step = 1;
heading = wb_inertial_unit_get_roll_pitch_yaw(imu)[2] * 180/M_PI;  
switch (corner) {
    case 2:
       heading_t = 135.0;
       set_Speed(left_speed, -left_speed);
       delay(11000);
       
       break; 
    case 3:
        heading_t = -90.0;
       if (heading >= -90.0 && heading <= 90.0) {
           set_Speed(left_speed, -left_speed);
           delay(10000);
       } else 
         {  set_Speed(-left_speed, left_speed);
           delay(8000);
         }    
       break; 
    case 4:
       heading_t = 45.0; 
      if (heading >= -90 && heading <= 90 ){
       set_Speed(-left_speed, left_speed);
       delay(15000);
       } else 
       {
       set_Speed(left_speed, -left_speed);
       delay(15000);
       }
       break;
}// switchcorner 


  stop();  
 
base_speed = 5.2; 
left_speed = right_speed = Vprevious_step = base_speed; //rad/s
printf("Parked. ================================================================================================================= \n");   
delay(2000); 
  
 
}//Park


void rest() {
    /*
   Emmanuel's og code modified */
   wb_motor_set_position(Motor1,-2.25077);
   
   delay(4500 );
   
   wb_motor_set_position(Motor2,0.148684);  
   wb_motor_set_position(Motor3,1.29885 ); 
   delay(2000);
   wb_motor_set_position(Motor4,0.66051);
   
   delay(2000);
   
   wb_motor_set_position(Motor6, 1.04657); 
   delay(3000);
   wb_motor_set_position(Motor5, -9.38165e-07); 

} //rest 


 
  
void PickandPlace(int beacon) {



   
 


} //p&p


/*
============================================================================================================================================================
 
 
Main. remember timestep is 16ms, and so the refresh rate for the motors and imu is 16ms
=============================================================================================================================================================*/
int main(int argc, char **argv) {
  wb_robot_init();  
  // find wheels
  fl_motor = wb_robot_get_device("FLMotor"); fr_motor = wb_robot_get_device("FRMotor"); 
  bl_motor = wb_robot_get_device("BLMotor"); br_motor = wb_robot_get_device("BRMotor");
  
  //Manipualotor 
  Motor1 = wb_robot_get_device("basemotor");   
  Motor2 = wb_robot_get_device("link2motor"); 
  Motor3 = wb_robot_get_device("link3motor"); 
  Motor4 = wb_robot_get_device("link4motor"); 
  Motor5 = wb_robot_get_device("link5motor"); 
  Motor6 = wb_robot_get_device("grippermotor");

  

  
  // find imu
  imu = wb_robot_get_device("inertial unit");
  wb_inertial_unit_enable(imu, TIME_STEP);
  
  //Hokuyo Utm 30lx lidar
  hokuyo = wb_robot_get_device("hokuyo_lidar");
  wb_lidar_enable(hokuyo, TIME_STEP);  bool lidar_control  =  true; 
  wb_lidar_enable_point_cloud(hokuyo);
  hokuyo_size = wb_lidar_get_horizontal_resolution(hokuyo);
  hokuyo_range = wb_lidar_get_max_range(hokuyo);
  hokuyo_fov = wb_lidar_get_fov(hokuyo);
  printf("number of Lidar distance points:=  %d \n", wb_lidar_get_number_of_points(hokuyo)); //Dean said print this 
  
  // intialise motors
  wb_motor_set_position( fl_motor, INFINITY); wb_motor_set_position( fr_motor, INFINITY); 
  wb_motor_set_position( bl_motor, INFINITY); wb_motor_set_position( br_motor, INFINITY); 
  
  //
  
        
   // initiate movement   
   left_speed = right_speed = Vprevious_step =  base_speed; 
   set_Speed(left_speed,right_speed); 
   bool squared =  false; double PID = 0.0; 
   
   int condition = 10; int condition_holder = 10; 
   bool First = false; heading_t = -45; //gets it moving in that general direction the first time. after that start using lidar
   int pitch_sign = 10; 
   
   int four = 1; int drivetime =0; 

   double startup_hiccups = 0; 
 

/* feedback loop
=============================================================================================================================================== */
while (wb_robot_step(TIME_STEP) != -1) {
 
      //Heading control 
       PID = PID_Controller();
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
          set_Speed(left_speed, right_speed); //careful note, we *do* set speed.
      } //if withinrange
      
      //actuation after correction (drive straight)
      squared = false; 
           
       if (First == false && (wb_robot_get_time()*1000  < 1000))
            pitch_i = wb_inertial_unit_get_roll_pitch_yaw(imu)[1] * 180/M_PI;
       /* Navigate to 1st hill, in this code, this will always be the southwest corner, -45 degrees */
      if (First == false && (wb_robot_get_time()*1000 > 11000)) {
          do{
              pitch_sign = FirstCorner(2000); 
           }while(pitch_sign > 0);
          
          condition = condition_holder = -1; //climbing 
          First = true; //I expect pitch to be negative bc of how much time I've let the simulation run and the initial heading_t
      } //if First 
          
      /* 
      * just to do with first corner
      * from here onwards vehicle should be driving towards first corner 
      * use lidar to get to mountain top       
      
      -1: climbiing
      0: stationary/stopped horizontal
      1: descending          */
         
      switch (condition) {
           case -1: //climbing 
                  if (lidar_control == false)
                  {wb_lidar_enable(hokuyo, TIME_STEP); wb_lidar_enable_point_cloud(hokuyo);
                      lidar_control  =  true;
                  }
                condition_holder = Highest();  
                 break; 
           case 0:
                   stop(); 
                   printf("Stopped================================================================================ \n"); 
                   delay(3000);                                
               
                  /* execute pick up and place*/
                  
               four++;
               Park(four); 
                 switch (four){
                      case 2:
                        heading_t = heading_prev = 135.0; //+180 not -180, code will take care of -180 rollaround, trust. 
                        drivetime = 50000;
                        break;
                      case 4:
                        heading_t = heading_prev= 45.0;
                        drivetime = 47000;
                        break; 
                      case 3:
                        heading_t = heading_prev = -90.0;
                        drivetime = 30000;
                        break;
                }// switch four 
               
                
               /* leaving hill top
               // Storing start time */      
               left_speed = right_speed = Vprevious_step =  base_speed; 
               set_Speed(left_speed,right_speed);
               startup_hiccups = wb_robot_get_time()*1000;     
               pitch_i = wb_inertial_unit_get_roll_pitch_yaw(imu)[1] * 180/M_PI;
               condition_holder = 1;
               break; 
          case 1:
                 if (lidar_control) { 
                  wb_lidar_disable(hokuyo); wb_lidar_disable_point_cloud(hokuyo);
                  lidar_control  =  false ; 
                  }
               
                   
                    pitch_f = wb_inertial_unit_get_roll_pitch_yaw(imu)[1] * 180/M_PI;
                  if (startup_hiccups < 1000 ) {
                      startup_hiccups = wb_robot_get_time()*1000; 
                      condition_holder = 1; 
                      break;
                   } 
                   
                  double current_time =  wb_robot_get_time()*1000; 
                  if ( current_time > startup_hiccups + drivetime){
                    //get pitch sign
                     pitch_sign = (pitch_f == 0) ? 0:pitch_f/fabs(pitch_f); 
                     heading = wb_inertial_unit_get_roll_pitch_yaw(imu)[2] * 180/M_PI;
                     
                      if (pitch_sign <= 0 && fabs(heading - heading_t)<=15) {
                         condition_holder =  -1;
                         pitch_i = wb_inertial_unit_get_roll_pitch_yaw(imu)[1] * 180/M_PI;
                         break; 
                     }
                 }  //if >  tartup_hiccups + 11000
                 break;
   }//switch (condition)
     condition = condition_holder;
     printf("condition %d ," , condition ); 
          
   
 
    
}; /* feedback while


 cleanup code here */

  wb_robot_cleanup();

  return 0;
}
