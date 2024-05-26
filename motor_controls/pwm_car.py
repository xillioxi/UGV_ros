# Simple demo of setting a PWM frequency and a channel's duty cycle.
# This example will set channel 3 to a specific duty cycle, but you would
# need additional control logic and hardware to fully control a DC motor.
# Author: Your Name
# License: Public Domain
from __future__ import division
import time

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(60)

# Define the PWM pulse for a neutral signal (1.5 ms)
# This value assumes a frequency of 60 Hz, with a period of 16666.67 microseconds.
# Since the PCA9685 has 4096 steps, we calculate the neutral position as follows:
neutral_pulse = int((1.3e-3) / (1.0/60) * 4096)

# Set channel 3 to the neutral pulse length
pwm.set_pwm(3, 0, neutral_pulse)

print('Setting channel 3 to neutral pulse width (1.5 ms), press Ctrl-C to quit...')
while True:
    try:
        # Here you could add code to change the PWM value based on some condition
        # to increase or decrease the speed of the motor.
        # For now, it will just maintain the neutral pulse width.
        time.sleep(1)
    except KeyboardInterrupt:
        # Stop the motor by setting it to neutral pulse width before exiting.
        pwm.set_pwm(3, 0, neutral_pulse)
        print("\nStopped motor on channel 3")
        break
