import math
import time
#from board import SCL, SDA
#import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

"""

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca = PCA9685(i2c)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca.frequency = 50

servo0 = servo.Servo(pca.channels[0], min_pulse=650, max_pulse=2800)
servo1 = servo.Servo(pca.channels[1], min_pulse=650, max_pulse=2800)
servo2 = servo.Servo(pca.channels[2], min_pulse=650, max_pulse=2800)

servo0.angle = 70
servo1.angle = 130
servo2.angle = 82

"""

arm1AngleC = 43
arm2AngleC = 150
arm1PointX = -117.017
arm1PointY = 57.939
arm1Length = 140
arm2Length = 90

targetX = 20
targetY = -30

acx = (arm1PointX-targetX)
acy = (arm1PointY-targetY)
ac = math.sqrt(acx**2+acy**2)
B = math.acos((ac**2-arm2Length**2-arm1Length**2)/(-2*arm2Length*arm1Length))
O = math.atan(abs(acy)/abs(acx))
A = math.asin(arm1Length*math.sin(B)/(ac))

B = math.degrees(B)
O = math.degrees(O)
A = math.degrees(A)


a = A-O
servo0angle = arm1AngleC + (180 - B)
servo1angle = arm2AngleC - a

#servo0.angle = servo0angle
#servo1.angle = servo1angle