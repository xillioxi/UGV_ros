import pca9685_sample
import moveservo_pca9685


def main():
    #Check if receving data from /camera/image_raw

    #Check if receiving data from /imu/data

    #Check if GPU has a fix or not

    #Check if receiving data from /orb_slam3/map_points

    #Check if motor directions are included

    #Check Servo
    moveservo_pca9685.initiate_servo()
    #Check Initiate the motor
    pca9685_sample.bootUp()
    
