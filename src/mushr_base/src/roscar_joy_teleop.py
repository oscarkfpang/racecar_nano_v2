#!/usr/bin/env python
#import roslib; roslib.load_manifest('ros_car')
import rospy
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float32, Float64
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import sys
import time

import Jetson.GPIO as GPIO

# Nano GPIO BOARD mode
RC_INPUT = 12  # BCM pin 18, BOARD pin 12

# define constant
V = 0.6

# define pin on PCA9685
ESC = 9
SERVO = 8

FREQ = 50 ## 1000
BIT = 4096

MIN_THROTTLE = 204 # @50Hz
MAX_THROTTLE = 430 # @50Hz
NEU_THROTTLE = (MAX_THROTTLE + MIN_THROTTLE) //2
DEAD_ZONE = (308, 338)

PW = 50 * 20000 #(Hz * us)

# define joystick stttings
Axis_X = 3 # steer
Axis_Y = 2 # throttle for logitech gamepad F310
Axis_Brake = 5 # brake
#Axis_Y = 3 # for PS4 joystick
ButtonStop = 2
ButtonReverse = 4 ##5


# define AckermannDrive settings
THROTTLE_SCALE = 2.0
STEER_SCALE = 0.7 ## 0.34
BRAKE_SCALE = 5.0

def throttle(joystick):
    if joystick < 0.0:  ## backward
        gas = (int) (DEAD_ZONE[0] - (-1) * joystick * (DEAD_ZONE[0] - MIN_THROTTLE))
    elif joystick > 0.0: ## forward
        gas = (int) (joystick * (MAX_THROTTLE - DEAD_ZONE[1]) + DEAD_ZONE[1])
    else:
        gas = NEU_THROTTLE
    rospy.loginfo("Throttle = "+str(gas))


def steer_isaac(turn):
    rospy.loginfo(float(turn.data))

    turn = (int)(turn.data * 500 + 1500)
    _steer = (int)(turn * BIT / (PW // FREQ))


def ReceiveJoystickMessage(joy):
    turn = float(joy.axes[Axis_X]) * STEER_SCALE
    #turn = (float(joy.axes[Axis_X]) + 1.0) / 2.0
    speed = (float(joy.axes[Axis_Y]) * -1.0 + 1.0) / 2.0 * THROTTLE_SCALE

    brake = (float(joy.axes[Axis_Brake]) * -1.0 + 1.0) / 2.0 * BRAKE_SCALE


    if(joy.buttons[ButtonReverse] == 0):
        speed = speed * -1

    rospy.loginfo("throttle = " + str(speed) + "  steer = " + str(turn))

    
    if joy.buttons[ButtonStop] == 1:
        rospy.loginfo("Stop")
        turn = 0
        speed = 0

    pub_teleop_msg(turn, speed)
    pubBrake.publish(brake)


def ReceiveIsaacJoystickMessage(joystick):
    _steer = float(joystick.axes[1])
    _throttle = float(joystick.axes[0])

    #throttle(_throttle)
    #steer(_steer)

def pub_teleop_msg(steering_angle, speed):
    ackermann_drive = AckermannDrive(steering_angle = steering_angle, speed = speed)
    ackermann_drive_stamp = AckermannDriveStamped(drive = ackermann_drive)
    ackermann_drive_stamp.header.frame_id = "joy_teleop"
    ackermann_drive_stamp.header.stamp = rospy.Time.now()
    #rospy.loginfo(ackermann_drive_stamp)
    pubTeleop.publish(ackermann_drive_stamp)


def shutdown():
    GPIO.cleanup()
    rospy.loginfo("Bye Bye! Shutdown RC_Joy_Mux Node")


if __name__ == '__main__':
    rospy.init_node('RC_Joy_Mux')
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(20) # 20 Hz

    car_name = rospy.get_param("~car_name", "/car")

    global activate
    activate = None

    GPIO.setmode(GPIO.BOARD) 
    GPIO.setup(RC_INPUT, GPIO.IN)
    rospy.loginfo("Listening to RC Interrupt signal")

    subJoystick = rospy.Subscriber('/joy', Joy, ReceiveJoystickMessage, queue_size=1)
    subJoystick_Isaac = rospy.Subscriber('/joystick', Joy, ReceiveIsaacJoystickMessage, queue_size=1)

    pubRCInterrupt = rospy.Publisher("/rc_interrupt", Bool, queue_size=1)
    pubTeleop = rospy.Publisher("{}/mux/ackermann_cmd_mux/output".format(car_name), AckermannDriveStamped, queue_size=1) 
    pubBrake = rospy.Publisher("/commands/motor/brake", Float32, queue_size=1)

    rospy.loginfo("Subscribed to joystick & Isaac joystick message.")

    try:
        while not rospy.is_shutdown():
            #activate = (GPIO.input(RC_INPUT) == GPIO.HIGH)
            #rospy.loginfo("gpio input is "+ str(activate))
            pubRCInterrupt.publish(GPIO.input(RC_INPUT) == GPIO.LOW)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass

    #finally:
        #shutdown()


