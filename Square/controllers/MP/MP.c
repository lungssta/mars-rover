/*

 * File:          manipulator_controller.c

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

#include <webots/Motor.h>

#include <stdio.h>

#include <time.h>

#include <math.h>

/*

 * You may want to add macros here.

 */

#define TIME_STEP 64





void delay(int milliseconds) { 

    // Storing start time 

    clock_t start_time = clock(); 

    // looping till required time is not achieved 

    while (wb_robot_get_time() *1000< start_time + milliseconds){

    wb_robot_step(TIME_STEP); 

    }

}

 WbDeviceTag Motor1;

 WbDeviceTag Motor2;

 WbDeviceTag Motor3;

 WbDeviceTag Motor4;

 WbDeviceTag Motor5;

 WbDeviceTag Motor6;

 

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



/*

 * This is the main program.

 * The arguments of the main function can be specified by the

 * "controllerArgs" field of the Robot node

 */

int main(int argc, char **argv) {

  /* necessary to initialize webots stuff */

  wb_robot_init();



  /*

   * You should declare here WbDeviceTag variables for storing

   * robot devices like this:

   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");

   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");

   */

  

  Motor1 = wb_robot_get_device("basemotor");   

  Motor2 = wb_robot_get_device("link2motor"); 

  Motor3 = wb_robot_get_device("link3motor"); 

  Motor4 = wb_robot_get_device("link4motor"); 

  Motor5 = wb_robot_get_device("link5motor"); 

  Motor6 = wb_robot_get_device("grippermotor");

  

  // initialise motors velocity // wb_motor_set_velocity(,);

    

  wb_motor_set_velocity(Motor1, 0.25);

  wb_motor_set_velocity(Motor2, 0.25);

  wb_motor_set_velocity(Motor3, 0.25); 

  wb_motor_set_velocity(Motor4, 0.25);

  wb_motor_set_velocity(Motor5, 0.25);

  wb_motor_set_velocity(Motor6, 0.25);  

  

   //  initialise position

  

   wb_motor_set_position(Motor1,-0.716*M_PI);

   delay(1000);

   

 //  delay(1000);

  wb_motor_set_position(Motor4,M_PI/6);

   wb_motor_set_position(Motor5,0);

   delay(1000);

   

   wb_motor_set_position(Motor5, M_PI/3); 

   delay(1000);

  

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

 

  

 

  

  

  

// Collect the first beacon (function)

  printf("position gripper above beacon\n"); 

 wb_motor_set_position(Motor3, M_PI/2 - (8*M_PI/180));

  wb_motor_set_position(Motor2,8*M_PI/180); 

  delay(3000);

   

 printf("open gripper\n"); 

  wb_motor_set_position(Motor6, -0.1);

//  delay(1000);

   

  

 //Engulfing the beacon

  /*for (int i = 1; i<8; i++){

   wb_motor_set_position(Motor4,M_PI/6 + 0.1*i*M_PI/6); 

   wb_motor_set_position(Motor5,M_PI/3 - 0.1*i*M_PI/6); 

  

  delay(1000); 

  

  } */

    





 



   delay(1000);

   

 //Gripping  

  wb_motor_set_position(Motor6, -0.01); 

   delay(1000); 



// picking up



  for (int i = 1; i<1; i++){

   wb_motor_set_position(Motor4,M_PI/6 - 0.1*i*M_PI/6); 

   wb_motor_set_position(Motor5,M_PI/3 + 0.1*i*M_PI/6); 

  }

  delay(1000); 
  
  
  rest(); 
};
  wb_robot_cleanup();

  return 0;  

  }

  