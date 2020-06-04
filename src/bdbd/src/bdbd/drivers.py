#!/usr/bin/env python
'''
    This ROS node combines many actuator activations that typically require python drivers.
'''
import rospy
import time

### robot wheel motors
# Adapted from https://github.com/dusty-nv/jetbot_ros/blob/master/scripts/jetbot_motors.py
# Adapted by R. Kent James <kent@caspia.com> for bdbd robot
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from bdbd.msg import MotorsRaw

BASE_SPEED = 0.4
MAX_SPEED = 1.0

# setup motor controller
motor_driver = Adafruit_MotorHAT(i2c_bus=1)

motor_left_ID = 1
motor_right_ID = 2

motor_left = motor_driver.getMotor(motor_left_ID)
motor_right = motor_driver.getMotor(motor_right_ID)

### pan/tilt hat
from bdbd.msg import PanTilt
from libpantilt.PCA9685 import PCA9685
panTilt = PCA9685()
panTilt.setPWMFreq(50)

# respeaker LEDs
from bdbd.libpy.respeaker.usb_pixel_ring_v2 import find
pixelring = find()

# 
### Motor functions
# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
    max_pwm = 255.0 # This allows for full 12 volt output
    #max_pwm = 255.0
    speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

    if motor_ID == 1:
        motor = motor_left
    elif motor_ID == 2:
        motor = motor_right
    else:
        rospy.logerr('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
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
    left = msg.left
    right = msg.right
    if left > MAX_SPEED or left < -MAX_SPEED or right > MAX_SPEED or right < -MAX_SPEED:
        rospy.logwarn_once('specified speed out of range, must be between {} and {}'.format(-MAX_SPEED, MAX_SPEED))
    left = min(max(-MAX_SPEED, left), MAX_SPEED)
    right = min(max(-MAX_SPEED, right), MAX_SPEED)
    rospy.loginfo('setting speed to {:5.3f}, {:5.3f}'.format(left, right))
    # TODO: somehow I got the polarity reversed on the motors
    if abs(left) < .05:
        left = 0.0
    if abs(right) < .05:
        right = 0.0
    set_speed(motor_left_ID,  -left)
    set_speed(motor_right_ID, -right)

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
        rospy.logerr(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)

### pan/tilt functions
class CenterPanTilt():
    ''' a PanTilt message preset to center pan/tilt '''
    pan = 90.0
    tilt = 45.0

def on_pantilt(msg):
    panTilt.setRotationAngle(1, max(0.0, min(180.0, msg.pan)))
    panTilt.setRotationAngle(0, max(0.0, min(90.0, msg.tilt)))

# pixelring functions
def on_pixelring(msg):
    command = msg.data
    if command == 'listen':
        pixelring.listen()
    elif command == 'speak':
        pixelring.speak()
    elif command == 'think':
        pixelring.think()
    elif command == 'spin':
        pixelring.spin()
    elif command == 'red':
        pixelring.set_color(r=255)
    elif command == 'blue':
        pixelring.set_color(b=255)
    elif command == 'purple':
        pixelring.set_color(b=255, r=255)
    elif command == 'off':
        pixelring.off()
    else:
        rospy.logwarn('Unexpected pixelring command <{}>'.format(command))

def main():
    # stop the motors as precaution
    all_stop()

    # initialize pan and tilt
    on_pantilt(CenterPanTilt)

    # set pixelring to default listen
    pixelring.listen()

    # setup ros node
    rospy.init_node('drivers')

    ### Motors
    rospy.Subscriber('motors/cmd_dir', String, on_cmd_dir)
    rospy.Subscriber('motors/cmd_raw', MotorsRaw, on_cmd_raw)
    rospy.Subscriber('motors/cmd_str', String, on_cmd_str)

    ### Pan/Tilt
    rospy.Subscriber('pantilt', PanTilt, on_pantilt)

    ### pixel ring
    rospy.Subscriber('pixelring', String, on_pixelring)

    # start running
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # stop motors before exiting
        all_stop()
        # center pan/tilt
        on_pantilt(CenterPanTilt)
        # pixelring listen
        pixelring.listen()

if __name__ == '__main__':
    main()
