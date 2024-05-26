from __future__ import division
import time
import rospy
from sensor_msgs.msg import Imu
import Adafruit_PCA9685

# Constants
SERVO_CHANNEL = 14
SERVO_MIN = 200
SERVO_MAX = 400
SERVO_NEUTRAL = 300
SERVO_FREQUENCY = 50
ORIENTATION_SENSITIVITY = 2.0  # Adjust this value to increase or decrease sensitivity

# Global variables
pwm = None
target_orientation = 0.0

def imu_callback(imu_msg):
    global target_orientation
    orientation = imu_msg.orientation
    # Extract the yaw angle from the quaternion
    # You may need to adjust this based on your IMU's coordinate frame
    target_orientation = orientation.z

def set_servo_pulse(channel, pulse):
    pulse_length = 1000000    # 1,000,000 us per second
    pulse_length //= SERVO_FREQUENCY
    print('{0}us per period'.format(pulse_length))
    pulse_length //= 4096     # 12 bits of resolution
    print('{0}us per bit'.format(pulse_length))
    pulse *= 1000
    pulse //= pulse_length
    pwm.set_pwm(channel, 0, pulse)

def main():
    global pwm
    rospy.init_node('servo_controller')
    rospy.Subscriber('/imu/data', Imu, imu_callback)

    pwm = Adafruit_PCA9685.PCA9685()
    pwm.set_pwm_freq(SERVO_FREQUENCY)

    rate = rospy.Rate(50)  # Increase the loop rate to 50 Hz for faster response
    while not rospy.is_shutdown():
        # Adjust servo PWM based on target orientation with increased sensitivity
        servo_pulse = SERVO_NEUTRAL + (target_orientation * ORIENTATION_SENSITIVITY * (SERVO_MAX - SERVO_MIN) / 2)
        set_servo_pulse(SERVO_CHANNEL, int(servo_pulse))
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
