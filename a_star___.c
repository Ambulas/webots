#include <math.h>
#include <stdio.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/keyboard.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 32
#define TARGET_POINTS_SIZE 6   //目标点数量
#define DISTANCE_TOLERANCE 0.1
#define MAX_SPEED 5
#define TURN_COEFFICIENT 6


enum XYZAComponents { X, Y, Z, ALPHA };
enum Sides { LEFT, RIGHT };

typedef struct _Vector {
  double u;
  double v;
} Vector;

static WbDeviceTag motors[2];
static WbDeviceTag gps;
static WbDeviceTag compass;
//static WbDeviceTag ds[3];

double speeds[2] = {0.0, 0.0};

/*设置目标点向量*/
static Vector targets[TARGET_POINTS_SIZE] = {
  //{2.5,-1},{-1.5,-1},{-2.5,-1},{-2.8,-2.7},{-2.8,-4.5},{-4.5,-4.5} //客厅到卧室A
 // {2.5,-1},{-1.5,-1},{-2.5,-1},{-3.3,-0.3},{-3.3,2.2},{-0.2,3.5} //客厅到卧室B
  //{2.5,-1},{-1.5,-1},{-2.5,-1},{-4.95,-1},{-5.7,2.89},{-8.83,4.01} //客厅到卧室C
  //{2.5,-1},{-1.5,-1},{-2.5,-1},{-4.95,-1},{-6.5,-1},{-8.35,-1}//客厅到卫生间
  //{8.7, -1.9},{7.2,-3.2},{3.3,-3.2},{1,-0.6},{-2.3,-0.6},{-3.5, 0.6},{-3.5,3.6},{-0.5,4.4}
  //{8,-3},{3.5,-3},{1,-1},{-2,-1},{-5.5,-1},{-5.5,3},{-9,4.5}
 // {4.6,2.2},{3.3,-1},{-3.3,-1},{-7.8,-1}  //厨房
   // {4,-5.3}, {4,-1}, {0.4,-1}, {-2.5,-1}, {-3.5,-1}, {-7.8,-1}
  {4.2, 2.2}
  };
static int current_target_index = 0;


static bool autopilot = true;
static bool old_autopilot = true;
static int old_key = -1;

static double modulus_double(double a, double m) {
  int div_i = (int)(a / m);
  double div_d = (double)div_i;
  double r = a - div_d * m;
  if (r < 0.0)
    r += m;
  return r;
}

// set left and right motor speed [rad/s]
static void robot_set_speed(double left, double right) {

   wb_motor_set_velocity(motors[0], left);
   wb_motor_set_velocity(motors[1], right);

}

static void check_keyboard() {
  double speeds[2] = {0.0, 0.0};

  int key = wb_keyboard_get_key();
  if (key >= 0) {
    switch (key) {
      case WB_KEYBOARD_UP:
        speeds[LEFT] = MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_DOWN:
        speeds[LEFT] = -MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_RIGHT:
        speeds[LEFT] = MAX_SPEED;
        speeds[RIGHT] = -MAX_SPEED;
        autopilot = false;
        break;
      case WB_KEYBOARD_LEFT:
        speeds[LEFT] = -MAX_SPEED;
        speeds[RIGHT] = MAX_SPEED;
        autopilot = false;
        break;
      case 'P':
        if (key != old_key) {  // perform this action just once
          const double *pos3D = wb_gps_get_values(gps);
          printf("position: {%f, %f}\n", pos3D[X], pos3D[Z]);
        }
        break;
      case 'A':
        if (key != old_key)  // perform this action just once
          autopilot = !autopilot;
        break;
     }
  }
  if (autopilot != old_autopilot)
  {
    old_autopilot = autopilot;
    if (autopilot)
      printf("auto control\n");
    else
      printf("manual control\n");
  }

  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
  old_key = key;
}

// ||v||
static double norm(const Vector *v) {
  return sqrt(v->u * v->u + v->v * v->v);
}

// v = v/||v||
static void normalize(Vector *v) {
  double n = norm(v);
  v->u /= n;
  v->v /= n;
}

// v = v1-v2
static void minus(Vector *v, const Vector *v1, const Vector *v2) {
  v->u = v1->u - v2->u;
  v->v = v1->v - v2->v;
}

// compute the angle between two vectors
// return value: [0, 2Pi[
static double angle(const Vector *v1, const Vector *v2) {
  return modulus_double(atan2(v2->v, v2->u) - atan2(v1->v, v1->u), 2.0 * M_PI);
}

// autopilot
// pass trough the predefined target positions
static void run_autopilot() {
  // prepare the speed array


  // read gps position and compass values
  const double *pos3D = wb_gps_get_values(gps);
  const double *north3D = wb_compass_get_values(compass);

  // compute the 2D position of the robo and its orientation
  Vector pos = {pos3D[X], pos3D[Z]};
  Vector north = {north3D[X], north3D[Z]};
  Vector front = {-north.u, north.v};

  // compute the direction and the distance to the target
  Vector dir;
  minus(&dir, &(targets[current_target_index]), &pos);
  double distance = norm(&dir);
  normalize(&dir);

  // compute the target angle
  double beta = angle(&front, &dir) - M_PI;

  // a target position has been reached
  if (distance < DISTANCE_TOLERANCE) {
    char index_char[3] = "th";
    if (current_target_index == 0)
      sprintf(index_char, "st");
    else if (current_target_index == 1)
      sprintf(index_char, "nd");
    else if (current_target_index == 2)
      sprintf(index_char, "rd");
    printf("%d%s target reached\n", current_target_index + 1, index_char);
    current_target_index++;
    //current_target_index %= TARGET_POINTS_SIZE;
  }
  // move the robot to the next target
  if(current_target_index == TARGET_POINTS_SIZE ){
      speeds[LEFT] = 0;
      speeds[RIGHT] = 0;
    }
  else {
    speeds[LEFT] = MAX_SPEED - M_PI + TURN_COEFFICIENT * beta;
    speeds[RIGHT] = MAX_SPEED - M_PI - TURN_COEFFICIENT * beta;
  }
  

  // set the motor speeds
  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
}

/*static void process_distance(float lf_wheel, float rt_wheel){
  
  if(lf_wheel > rt_wheel && lf_wheel > 50){
      
      speeds[LEFT] = 3.0;
      speeds[RIGHT] = 0.0;
  }
  else if(lf_wheel < rt_wheel && rt_wheel > 50){
      speeds[LEFT] = 0.0;
      speeds[RIGHT] = 3.0;
  }
  
  robot_set_speed(speeds[LEFT], speeds[RIGHT]);
  
}
*/
int main(int argc, char *argv[]) {
  // initialize webots communication
  wb_robot_init();

  // print user instructions
  printf("\f");
  printf("You can drive this robot:\n");
  printf("Select the 3D window and use cursor keys:\n");
  printf("Press 'A' to return to the autopilot mode\n");
  printf("Press 'P' to get the robot position\n");
  printf("\n");

  wb_robot_step(1000);

  const char *names[2] = {"motor3",  "motor4"};

  // get motor tags
  int i;
  for (i = 0; i < 2; i++) {
    motors[i] = wb_robot_get_device(names[i]);
    wb_motor_set_position(motors[i], INFINITY);
  }

  // get gps tag and enable
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  // get compass tag and enable
  compass = wb_robot_get_device("compass");
  wb_compass_enable(compass, TIME_STEP);

  // enable keyboard
  wb_keyboard_enable(TIME_STEP);

  // start forward motion
  robot_set_speed(MAX_SPEED, MAX_SPEED);
  
  // star hongwai

   // char ds_names[3][10] = { "ds_left", "ds_right" ,"ds_left_1"};
  //for (int i = 0; i < 3; i++) {
   //     ds[i] = wb_robot_get_device(ds_names[i]);
  //      wb_distance_sensor_enable(ds[i], TIME_STEP);
   // }

  // main loop
  while (wb_robot_step(TIME_STEP) != -1) {
    check_keyboard();
    if (autopilot){
      
      //float  lf_wheel = wb_distance_sensor_get_value(ds[0]);
     // float  rt_wheel = wb_distance_sensor_get_value(ds[1]);
      
      //float  lf_wheel_1 = wb_distance_sensor_get_value(ds[2]);
      
      
      //if(lf_wheel >20 || rt_wheel >20){
        //for(int i = 0; i<100; i++){
        //  process_distance(lf_wheel, rt_wheel);
        // }
       // if(lf_wheel_1 >30){
       //   speeds[LEFT] = 5.0;
       //   speeds[RIGHT] = 5.0;
       //   robot_set_speed(speeds[LEFT], speeds[RIGHT]);
       // }
      //}
      //else{
        run_autopilot();
      //}
    
     }
  }

  wb_robot_cleanup();

  return 0;
}
