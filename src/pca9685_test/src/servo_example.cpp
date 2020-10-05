#include <ros/ros.h>
#include "pca9685_library/PCA9685.h"

#include <iostream>
#include <stdio.h>
#include <thread>         // std::thread
#include <vector>         // std::vector
#include <unistd.h>
#include <string>
#include <string.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>


#define MIN_PULSE_WIDTH 900
#define MAX_PULSE_WIDTH 2100
#define FREQUENCY 50
#define SERVO_PIN 8

#define STEER_AXIS 0

int offset = 0;

PCA9685 pwm;

//Declaration of Functions used ==================================
int pwmwrite(int& angle, PCA9685 pwm, int& channel);


//def map(self, x, in_min, in_max, out_min, out_max):
int map_val (int x, int in_min, int in_max, int out_min, int out_max) {
        return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
void servoCallback(const std_msgs::Float64::ConstPtr& servo);


//def _angle_to_analog(self, angle):
int angleToAnalog(int angle) {
      float pulse_wide;
      int analog_value;
      
      pulse_wide = map_val(angle,0,180,MIN_PULSE_WIDTH,MAX_PULSE_WIDTH);
      analog_value = int(float(pulse_wide) /  1000000 * FREQUENCY * 4096);
      return (analog_value);
}

int pwmwrite(int angle, PCA9685 pwm, int channel) {
    int val = 0;

    if (angle > 180) {
       angle = 179;
    }
    if (angle < 0) {
       angle = 1;
    }
    
    val = angleToAnalog(angle);
    //not sure what offset does
    val += offset;

    //setPWM(self, channel, on, off
    //channel: The channel that should be updated with the new values (0..15)
    //on: The tick (between 0..4095) when the signal should transition from low to high
    //off:the tick (between 0..4095) when the signal should transition from high to low
    
    pwm.setPWM(channel,0,val);
    //usleep(30);
    std::cout << "Channel: " << channel << "\tSet to angle: " << angle << "\tVal: " << val << std::endl;
    return(0);
}


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
   int steer_angle = (int) (90.0 * joy->axes[STEER_AXIS] + 90.0);
   ROS_INFO("Steer Angle = %f", float(joy->axes[STEER_AXIS]));
   pwmwrite(steer_angle, pwm, SERVO_PIN);

}

void servoCallback(const std_msgs::Float64::ConstPtr& servo)
{
   float servo_val = servo->data;
   int steer_angle = (int) (servo_val * 180);
   ROS_INFO("Steer Angle = %d", steer_angle);
   pwmwrite(steer_angle, pwm, SERVO_PIN);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "servo_example");
    ros::NodeHandle nh;
    ROS_INFO("Receive Servo Command for PCA9685");


    ros::NodeHandle private_nh("~");

    ros::Subscriber joy_sub_ = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
    ////ros::Subscriber servo_sub_ = nh.subscribe("commands/servo/position", 10, servoCallback);


	//make sure you use the right address values.
	//PCA9685 pwm;
	pwm.init(1,0x40);
        //usleep(1000 * 100);
    ros::Duration(1.0).sleep();

	pwm.setPWMFreq (FREQUENCY);
	//usleep(1000 * 1000);
    ros::Duration(1.0).sleep();
    ROS_INFO("Paratmeters have been set! Waiting for joystick command ....");

    //ROS_INFO("30-degree");
//	pwmwrite(40, pwm, SERVO_PIN);
  //  ros::Duration(2.0).sleep();

    //ROS_INFO("90-degree");
//	pwmwrite(90, pwm, SERVO_PIN);
  //  ros::Duration(2.0).sleep();

    //ROS_INFO("150-degree");
//	pwmwrite(140, pwm, SERVO_PIN);
  //  ros::Duration(1.0).sleep();
    //ROS_INFO("90-degree");
//	pwmwrite(90, pwm, SERVO_PIN);

  //  ROS_INFO("end...");


    ros::spin();


	return 0;
} 
