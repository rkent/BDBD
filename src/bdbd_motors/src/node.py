#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from bdbd_motors.msg import MotorsRaw

# Adapted from https://github.com/dusty-nv/jetbot_ros/blob/master/scripts/jetbot_motors.py
# Adapted by R. Kent James <kent@caspia.com> for bdbd robot

BASE_SPEED = 0.4
MAX_SPEED = 1.0

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
    max_pwm = 115.0
    speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

    if motor_ID == 1:
        motor = motor_left
    elif motor_ID == 2:
        motor = motor_right
    else:
        rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
        return
    
    motor.setSpeed(speed)

    if value > 0:
        motor.run(Adafruit_MotorHAT.FORWARD)
    else:
        motor.run(Adafruit_MotorHAT.BACKWARD)


# stops all motors
def all_stop():
    motor_left.setSpeed(0)
    motor_right.setSpeed(0)

    motor_left.run(Adafruit_MotorHAT.RELEASE)
    motor_right.run(Adafruit_MotorHAT.RELEASE)


# directional commands (degree, speed)
def on_cmd_dir(msg):
    rospy.loginfo(rospy.get_caller_id() + ' cmd_dir=%s', msg.data)
    rospy.logwarn_once('cmd_dir not implemented')

# raw L/R motor commands (speed, speed)
def on_cmd_raw(msg):
    rospy.loginfo(rospy.get_caller_id() + ' cmd_raw=%s', type(msg))
    left = msg.left
    right = msg.right
    if left > MAX_SPEED or left < -MAX_SPEED or right > MAX_SPEED or right < -MAX_SPEED:
        rospy.logwarn_once('specified speed out of range, must be between {} and {}'.format(-MAX_SPEED, MAX_SPEED))
    left = min(max(-MAX_SPEED, left), MAX_SPEED)
    right = min(max(-MAX_SPEED, right), MAX_SPEED)
    rospy.loginfo('setting speed to {:5.3f}, {:5.3f}'.format(left, right))
    set_speed(motor_left_ID,  left)
    set_speed(motor_right_ID, right)

# simple string commands (left/right/forward/backward/stop)
def on_cmd_str(msg):
    rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

    if msg.data.lower() == "left":
        set_speed(motor_left_ID,  -BASE_SPEED)
        set_speed(motor_right_ID,  BASE_SPEED) 
    elif msg.data.lower() == "right":
        set_speed(motor_left_ID,   BASE_SPEED)
        set_speed(motor_right_ID, -BASE_SPEED) 
    elif msg.data.lower() == "forward":
        set_speed(motor_left_ID,   BASE_SPEED)
        set_speed(motor_right_ID,  BASE_SPEED)
    elif msg.data.lower() == "backward":
        set_speed(motor_left_ID,  -BASE_SPEED)
        set_speed(motor_right_ID, -BASE_SPEED)  
    elif msg.data.lower() == "stop":
        all_stop()
    else:
        rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)


# initialization
if __name__ == '__main__':

    # setup motor controller
    motor_driver = Adafruit_MotorHAT(i2c_bus=1)

    motor_left_ID = 1
    motor_right_ID = 2

    motor_left = motor_driver.getMotor(motor_left_ID)
    motor_right = motor_driver.getMotor(motor_right_ID)

    # stop the motors as precaution
    all_stop()

    # setup ros node
    rospy.init_node('bdbd_motors')
    
    rospy.Subscriber('~cmd_dir', String, on_cmd_dir)
    rospy.Subscriber('~cmd_raw', MotorsRaw, on_cmd_raw)
    rospy.Subscriber('~cmd_str', String, on_cmd_str)

    # start running
    rospy.spin()

    # stop motors before exiting
    all_stop()

