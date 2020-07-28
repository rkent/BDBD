# Test of center point calculations

# Adapted from https://github.com/dusty-nv/jetbot_ros/blob/master/scripts/jetbot_motors.py
# Adapted by R. Kent James <kent@caspia.com> for bdbd robot
from Adafruit_MotorHAT import Adafruit_MotorHAT

# setup motor controller
motor_driver = Adafruit_MotorHAT(i2c_bus=1)

motor_left_ID = 1
motor_right_ID = 2

motor_left = motor_driver.getMotor(motor_left_ID)
motor_right = motor_driver.getMotor(motor_right_ID)

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
    #max_pwm = 115.0
    max_pwm = 255.0 # 12 volts
    speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

    if motor_ID == motor_left_ID:
        motor = motor_left
    else:
        motor = motor_right
    
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

if __name__ == '__main__':
    import time
    try:
        left = 0.0
        right = 0.4
        print('motor on')
        # negative because of BDBD wiring
        set_speed(motor_left_ID,  -left)
        set_speed(motor_right_ID, -right)
        time.sleep(3)
    except:
        pass
    all_stop()
