/*
 * File:          control.c
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
#include <webots/motor.h>

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
  
  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   
     WbDeviceTag motors[4];
  char motors_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (int i = 0; i < 4; i++) {
    motors[i] = wb_robot_get_device(motors_names[i]);
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i],0);
  }

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
    double left_speed = 2.0;
    double right_speed = 2.0;
    /* Process sensor data here */
    wb_motor_set_velocity(motors[0],left_speed);
    wb_motor_set_velocity(motors[1],right_speed);
    wb_motor_set_velocity(motors[2],left_speed);
    wb_motor_set_velocity(motors[3],right_speed);
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

