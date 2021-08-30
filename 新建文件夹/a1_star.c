#include<math.h>
#include<stdio.h>
#include<webots/compass.h>
#include<webots/gps.h>
#include<webots/keyboard.h>
#include<webots/motor.h>
#include<webots/robot.h>
#include<webots/distance_sensor.h>

#define TIME_STEP 64
#define TARGET_POINTS_SIZE 1 //目标点数量
#define DISTANCE_TOLERANCE 0.1
#define MAX_SPEED 5.0
#define TURN_COEFFICIENT 6

enum XYZAComponents { X, Y, Z, ALPHA };//定义了一个向量类型
enum Sides { LEFT, RIGHT };
WbDeviceTag ds[2];
typedef struct _Vector 
{
	double u;
	double v;
} 
Vector;


static WbDeviceTag motors[2];
static WbDeviceTag gps;
static WbDeviceTag compass;

double speeds[2] = {0.0, 0.0}; 
//int flag=1;


int current_target_index = 0;
bool autopilot = true;
bool old_autopilot = true;
int old_key = -1;


double modulus_double(double a, double m) //定义两个向量之间的弧度，确保在（0.2pi）之间
{
	int div_i = (int)(a / m);
	double div_d = (double)div_i;
	double r = a - div_d * m;
	if (r < 0.0)
	r += m;
	return r;
}

static Vector targets[TARGET_POINTS_SIZE] = {
 //{2.5,-1},{-1.5,-1},{-2.5,-1},{-2.8,-2.7},{-2.8,-4.5},{-4.5,-4.5} //客厅到卧室A
//{2.5,-1},{-1.5,-1},{-2.5,-1},{-3.3,-0.3},{-3.3,2.2},{-0.2,3.5} //客厅到卧室B
//{2.5,-1},{-1.5,-1},{-2.5,-1},{-4.95,-1},{-5.7,2.89},{-8.83,4.01} //客厅到卧室C
//{2.5,-1},{-1.5,-1},{-2.5,-1},{-4.95,-1},{-6.5,-1},{-8.35,-1}//客厅到卫生间
//{8.7, -1.9},{7.2,-3.2},{3.3,-3.2},{1,-0.6},{-2.3,-0.6},{-3.5, 0.6},{-3.5,3.6},{-0.5,4.4}
//{8,-3},{3.5,-3},{1,-1},{-2,-1},{-5.5,-1},{-5.5,3},{-9,4.5}
// {4.6,2.2},{3.3,-1},{-3.3,-1},{-7.8,-1}  //厨房
// {4,-5.3}, {4,-1}, {0.4,-1}, {-2.5,-1}, {-3.5,-1}, {-7.8,-1}
{4.2, 2.2}
};
static void robot_set_speed(double left, double right)
{
	
	wb_motor_set_velocity(motors[0], left);
	wb_motor_set_velocity(motors[1], right);

}

char ds_names[2][10]={"ds_left","ds_right"};

static void check_keyboard()
{

	int key = wb_keyboard_get_key();

	if(wb_keyboard_get_key>=0)
	{
	switch (key) 
	{
		case WB_KEYBOARD_UP:
			speeds[LEFT]  += 0.5*MAX_SPEED;
			speeds[RIGHT] += 0.5*MAX_SPEED;
			autopilot = false;
			break;
		case WB_KEYBOARD_DOWN:
			speeds[LEFT] -= 0.5*MAX_SPEED;
			speeds[RIGHT] -= 0.5*MAX_SPEED;
			autopilot = false;
			break;
		case WB_KEYBOARD_RIGHT:
			speeds[LEFT] += 0.5*MAX_SPEED;
			speeds[RIGHT] -= 0.5*MAX_SPEED;
			autopilot = false;
			break;
		case WB_KEYBOARD_LEFT:
			speeds[LEFT] -= 0.5*MAX_SPEED;
			speeds[RIGHT] += 0.5*MAX_SPEED;
			autopilot = false;
			break;
		case 'P':
			if (key != old_key) 
			{  // perform this action just once
				const double *pos3D = wb_gps_get_values(gps);
				printf("position: {%f, %f}\n", pos3D[X], pos3D[Z]);
			}
			break;
		case 'A':
			if (key != old_key)  // perform this action just once
			{
				autopilot = !autopilot;	
			}
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
static double norm(const Vector *v) 
{
	return sqrt(v->u * v->u + v->v * v->v);
}

// v = v/||v||
static void normalize(Vector *v)
{
	double n = norm(v);
	v->u /= n;
	v->v /= n;
}

// v = v1-v2
static void minus(Vector *v, const Vector *v1, const Vector *v2) 
{
	v->u = v1->u - v2->u;
	v->v = v1->v - v2->v;
}

// 计算两个向量之间的角度// return value: [0, 2Pi]
static double angle(const Vector *v1, const Vector *v2) 
{
	return modulus_double(atan2(v2->v, v2->u) - atan2(v1->v, v1->u), 2.0 * M_PI);
}

// 自动驾驶// 通过预定义的目标位置
static void run_autopilot() 
{
	// 准备速度数组
	// read gps position and compass values
	const double *pos3D = wb_gps_get_values(gps);
	const double *north3D = wb_compass_get_values(compass);

	//计算机器人的2D位置及其方向
	Vector pos = {pos3D[X], pos3D[Z]};
	Vector north = {north3D[X], north3D[Z]};
	Vector front = {-north.u, north.v};

	// 计算方向和到目标的距离
	Vector dir;
	minus(&dir, &(targets[current_target_index]), &pos);
	double distance = norm(&dir);
	normalize(&dir);

	// 计算目标角度
	double beta = angle(&front, &dir) - M_PI;

	// 达到目标位置
	if (distance < DISTANCE_TOLERANCE) 
	{
		char index_char[3] = "th";
		if (current_target_index == 0)
		sprintf(index_char, "st");
		else if (current_target_index == 1)
		sprintf(index_char, "nd");
		else if (current_target_index == 2)
		sprintf(index_char, "rd");
		printf("%d%s target reached\n", current_target_index + 1, index_char);
		current_target_index++;//current_target_index %= TARGET_POINTS_SIZE;
	}
	// move the robot to the next target
	if(current_target_index == TARGET_POINTS_SIZE )
	{
		speeds[LEFT] = 0;
		speeds[RIGHT] = 0;
	}
	else 
	{
		speeds[LEFT] = 0.5*MAX_SPEED - M_PI + TURN_COEFFICIENT * beta;
		speeds[RIGHT] = 0.5*MAX_SPEED - M_PI - TURN_COEFFICIENT * beta;
	}

	// set the motor speeds
	robot_set_speed(speeds[LEFT], speeds[RIGHT]);
}


int main(int argc,char **argv)
{	
	wb_robot_init(); 
	int i,t,q;
	double ds_values[2];
	const char *names[2] = {"motor3",  "motor4"};
	for (i=0;i<2;i++)
	{
		ds[i]=wb_robot_get_device(ds_names[i]);
		wb_distance_sensor_enable(ds[i],TIME_STEP);
	}
	
	for (t = 0; t < 2; t++) 
	{
		motors[t] = wb_robot_get_device(names[t]);
		wb_motor_set_position(motors[t], INFINITY);
	}
	/* WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");//初始化电机
	WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);*/
	//加motorde get_device


	//double ds_values[2];
	// for (i = 0; i < 2; i++)
	//  {
	//	ds_values[i] = wb_distance_sensor_get_value(ds[i]);
	//  }      

	printf("\f");
	printf("You can drive this robot:\n");
	printf("Select the 3D window and use cursor keys:\n");
	printf("Press 'A' to return to the autopilot mode\n");
	printf("Press 'P' to get the robot position\n");
	printf("\n");

	wb_robot_step(1000);

	

	//WbDeviceTag wheels[4];
	//char wheels_names[4][8]={"wheel1","wheel2","wheel3","wheel4"};
	//for (i=0;i<4;i++)
	//{
	//  wheels[i] = wb_robot_get_device(wheels_names[i]);
	//  wb_motor_set_position(wheels[i], INFINITY);
	//} 
	// get gps tag and enable
	gps = wb_robot_get_device("gps");
	wb_gps_enable(gps, TIME_STEP);

	// get compass tag and enable
	compass = wb_robot_get_device("compass");
	wb_compass_enable(compass, TIME_STEP);

	// enable keyboard
	wb_keyboard_enable(TIME_STEP);

	// start forward motion
	robot_set_speed(MAX_SPEED,MAX_SPEED);
	//run_autopilot();//自动驾驶
	while(wb_robot_step(TIME_STEP) != -1)
	{
		check_keyboard();//按键检测

		for (q = 0; q < 2; q++)
		{
			ds_values[q] = wb_distance_sensor_get_value(ds[q]);//得到距离值
		}  

		if(ds_values[0]<100.0||ds_values[1]<100.0)//距离判断
		{
		         // if (flag==1)
		          // {
		           
			autopilot = false;//退出自动驾驶
			speeds[LEFT] += 0.5*MAX_SPEED;//转弯
			speeds[RIGHT] -= 0.5*MAX_SPEED;//转弯
			robot_set_speed(speeds[LEFT], speeds[RIGHT]);//设置速度
			    //if (flag==1)
		             //  {
			   wb_robot_step(1000);
			   // flag=0;
			// }
			speeds[LEFT] = 0.5*MAX_SPEED;//转弯
			speeds[RIGHT] = 0.5*MAX_SPEED;//转弯
			robot_set_speed(speeds[LEFT], speeds[RIGHT]);
			wb_robot_step(500);
			   // run_autopilot();
			//delay_ms(1000);
			//speeds[LEFT] = 0.5*MAX_SPEED;
			//speeds[RIGHT] = 0.5*MAX_SPEED;
			autopilot = true;//是否自动驾驶标志位
			
		//可加直行指令
			if(autopilot)
			{
				run_autopilot();//自动驾驶
			}
		}
		else
		{
			run_autopilot();
		}	

	}
	wb_robot_cleanup();// cleanup the Webots API

	return 0;

}
