#!/usr/bin/env python
'''
    This ROS node combines many actuator activations that typically require python drivers.
'''
import rospy
import time
import traceback
import threading

### robot wheel motors
# Adapted from https://github.com/dusty-nv/jetbot_ros/blob/master/scripts/jetbot_motors.py
# Adapted by R. Kent James <kent@caspia.com> for bdbd robot
from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from bdbd.msg import MotorsRaw
from bdbd_common.msg import PanTilt
from libpantilt.PCA9685 import PCA9685
from bdbd.libpy.respeaker.usb_pixel_ring_v2 import find
try:
    from Queue import Queue, Empty
except:
    from queue import Queue, Empty

import tf
tf_br = tf.TransformBroadcaster()

# these are set to slightly off center to force initial motion
pan = 89.1
tilt = 44.1
PANTILT_DP = 2 # maximum degrees per interval for pan, half this for tilt
D_TO_R = 3.1415926535 / 180. # degrees to radians
PANTILT_RATE = 100

# See RKJ 2021-03-04 p 78 for angle corrections
PAN_CORR = 2.8
TILT_CORR = -2.3

motor_left_ID = 1
motor_right_ID = 2

def main():

    panTiltQueue = Queue()
    #
    ### Motor functions
    #
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
        
        MAX_ERRORS = 4
        success = False
        for count in range(MAX_ERRORS):
            try:
                motor.setSpeed(speed)
                if value > 0:
                    motor.run(Adafruit_MotorHAT.FORWARD)
                else:
                    motor.run(Adafruit_MotorHAT.BACKWARD)
                success = True
            except:
                rospy.logwarn('Motor setSpeed error, retrying')
            if success:
                break
        if not success:
            rospy.logerr('Motor setSpeed failed')

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
        start = time.time()
        try:
            left = msg.left
            right = msg.right
            if left > MAX_SPEED or left < -MAX_SPEED or right > MAX_SPEED or right < -MAX_SPEED:
                rospy.logwarn_once('specified speed out of range, must be between {} and {}'.format(-MAX_SPEED, MAX_SPEED))
            left = min(max(-MAX_SPEED, left), MAX_SPEED)
            right = min(max(-MAX_SPEED, right), MAX_SPEED)
            # TODO: somehow I got the polarity reversed on the motors
            if abs(left) < .05:
                left = 0.0
            if abs(right) < .05:
                right = 0.0
            set_speed(motor_left_ID,  -left)
            set_speed(motor_right_ID, -right)
            #rospy.loginfo('set speed to {:5.3f}, {:5.3f} lag ms: {:6.3f}'.format(left, right, (time.time() - start) * 1000.))
        except:
            rospy.logerr(traceback.format_exc())

    # simple string commands (left/right/forward/backward/stop)
    def on_cmd_str(msg):
        try:
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
        except:
            rospy.logerr(traceback.format_exc())

    ### pan/tilt functions
    class CenterPanTilt():
        ''' a PanTilt message preset to center pan/tilt '''
        pan = 90.0
        tilt = 45.0

    # this has been slew limited on the theory that rapid slews are causing power glitches that is crashing the nano
    def do_pantilt(panTiltRate):
        global pan
        global tilt

        while True:
            panTiltMsg = panTiltQueue.get()
            # use None to exit thread
            if panTiltMsg is None:
                break
            rospy.loginfo('PanTilt msg {} {} current pan,tilt {} {}'.format(panTiltMsg.pan, panTiltMsg.tilt, pan, tilt))
            target_pan = panTiltMsg.pan
            target_tilt = panTiltMsg.tilt
            while pan != target_pan or tilt != target_tilt:
                if pan < target_pan:
                    pan = min(target_pan, pan + PANTILT_DP)
                else:
                    pan = max(target_pan, pan - PANTILT_DP)

                if tilt < target_tilt:
                    tilt = min(target_tilt, tilt + PANTILT_DP/2)
                else:
                    tilt = max(target_tilt, tilt - PANTILT_DP/2)

                try:
                    panTilt.setRotationAngle(1, max(0.0, min(180.0, pan)))
                    panTilt.setRotationAngle(0, max(0.0, min(90.0, tilt)))

                except:
                    rospy.logerr(traceback.format_exc())

                pantilt_tf_cb(None)
                panTiltRate.sleep()

    def pantilt_tf_cb(timerEvent):
        # calculate rotations for transform
        # 1) pan
        qpan_rot = tf.transformations.quaternion_from_euler(0., 0., (pan + PAN_CORR - 90.) * D_TO_R)
        tf_br.sendTransform((0, 0, 0), qpan_rot, rospy.Time.now(), 'pantilt_pan', 'pantilt_base')
        # 2) tilt
        qtilt_rot = tf.transformations.quaternion_from_euler(0., (tilt + TILT_CORR - 45.) * D_TO_R, 0.0)
        tf_br.sendTransform((0, 0, 0), qtilt_rot, rospy.Time.now(), 'pantilt_tilt', 'pantilt_axis')

    def on_pantilt(msg):
        print('on_pantilt')
        panTiltQueue.put(msg)

    # pixelring functions
    def on_pixelring(msg):
        try:
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
        except:
            rospy.logerr(traceback.format_exc())

    # setup ros node
    rospy.init_node('drivers')
    BASE_SPEED = 0.4
    MAX_SPEED = 1.0

    # setup motor controller
    try:
        motor_driver = Adafruit_MotorHAT(i2c_bus=1)

        motor_left = motor_driver.getMotor(motor_left_ID)
        motor_right = motor_driver.getMotor(motor_right_ID)
        # stop the motors as precaution
        all_stop()

    except:
        rospy.logerr(traceback.format_exc())

    ### pan/tilt hat
    try:
        panTilt = PCA9685()
        panTilt.setPWMFreq(50)
    except:
        rospy.logerr(traceback.format_exc())

    panTiltQueue.put(CenterPanTilt)
    panTiltRate = rospy.Rate(PANTILT_RATE)
    panTiltThread = threading.Thread(target=do_pantilt, args=[panTiltRate])
    panTiltThread.start()

    # respeaker LEDs
    try:
        pixelring = find()
        # set pixelring to default listen
        pixelring.listen()
    except:
        rospy.logwarn('pixelring publish failed')
        rospy.logwarn(traceback.format_exc())

    ### Motors
    rospy.Subscriber('motors/cmd_dir', String, on_cmd_dir, tcp_nodelay=True)
    rospy.Subscriber('motors/cmd_raw', MotorsRaw, on_cmd_raw, queue_size=1, tcp_nodelay=True)
    rospy.Subscriber('motors/cmd_str', String, on_cmd_str, tcp_nodelay=True)

    ### Pan/Tilt
    rospy.Subscriber('pantilt', PanTilt, on_pantilt, tcp_nodelay=True)

    ### pixel ring
    rospy.Subscriber('pixelring', String, on_pixelring, tcp_nodelay=True)

    # start running
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # stop motors before exiting
        all_stop()
        # center pan/tilt
        panTiltQueue.put(CenterPanTilt)
        # pixelring listen
        pixelring.listen()
        rospy.sleep(1)

if __name__ == '__main__':
    main()
