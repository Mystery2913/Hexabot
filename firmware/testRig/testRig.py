import math
import time
from board import SCL, SDA
import busio

# Import the PCA9685 module. Available in the bundle and here:
#   https://github.com/adafruit/Adafruit_CircuitPython_PCA9685
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)

# Create a simple PCA9685 class instance.
pca0 = PCA9685(i2c, address=64)
pca1 = PCA9685(i2c, address=65)
# You can optionally provide a finer tuned reference clock speed to improve the accuracy of the
# timing pulses. This calibration will be specific to each board and its environment. See the
# calibration.py example in the PCA9685 driver.
# pca = PCA9685(i2c, reference_clock_speed=25630710)
pca0.frequency = 50
pca1.frequency = 50

servo0 = servo.Servo(pca0.channels[0], min_pulse=650, max_pulse=2800)
servo1 = servo.Servo(pca0.channels[1], min_pulse=650, max_pulse=2800)
servo2 = servo.Servo(pca0.channels[2], min_pulse=650, max_pulse=2800)
servo3 = servo.Servo(pca0.channels[3], min_pulse=650, max_pulse=2800)
servo4 = servo.Servo(pca0.channels[4], min_pulse=650, max_pulse=2800)
servo5 = servo.Servo(pca0.channels[5], min_pulse=650, max_pulse=2800)
servo6 = servo.Servo(pca0.channels[6], min_pulse=650, max_pulse=2800)
servo7 = servo.Servo(pca0.channels[7], min_pulse=650, max_pulse=2800)
servo8 = servo.Servo(pca0.channels[8], min_pulse=650, max_pulse=2800)

servo9 = servo.Servo(pca1.channels[0], min_pulse=650, max_pulse=2800)
servo10 = servo.Servo(pca1.channels[1], min_pulse=650, max_pulse=2800)
servo11 = servo.Servo(pca1.channels[2], min_pulse=650, max_pulse=2800)
servo12 = servo.Servo(pca1.channels[3], min_pulse=650, max_pulse=2800)
servo13 = servo.Servo(pca1.channels[4], min_pulse=650, max_pulse=2800)
servo14 = servo.Servo(pca1.channels[5], min_pulse=650, max_pulse=2800)
servo15 = servo.Servo(pca1.channels[6], min_pulse=650, max_pulse=2800)
servo16 = servo.Servo(pca1.channels[7], min_pulse=650, max_pulse=2800)
servo17 = servo.Servo(pca1.channels[8], min_pulse=650, max_pulse=2800)

leg0 = [servo0, servo1, servo2]
leg1 = [servo3, servo4, servo5]
leg2 = [servo6, servo7, servo8]
leg3 = [servo9, servo10, servo11]
leg4 = [servo12, servo13, servo14]
leg5 = [servo15, servo16, servo17]

legs = [leg0, leg1, leg2, leg3, leg4, leg5]

servo0.angle = 70
#\ servo1.angle = 130
# servo2.angle = 82

arm1AngleC = 43
arm2AngleC = 150
arm2PointY = 57.939
arm3PointX = -215.517
arm3PointZ = 0
arm2To3X = 98.50
arm1Length = 140
arm2Length = 90

def moveToCartesian(leg, targetX, targetY, targetZ):

    Zangle = math.atan(targetZ/(abs(arm3PointX)+targetX))

    totalArm3Length = math.sqrt((abs(arm3PointX)+targetX)**2+(targetZ)**2)
    underAC = totalArm3Length - arm2To3X
    Zangle = math.degrees(Zangle)


    acy = (arm2PointY-targetY)
    ac = math.sqrt(underAC**2+acy**2)
    B = math.acos((ac**2-arm2Length**2-arm1Length**2)/(-2*arm2Length*arm1Length))
    O = math.atan(abs(acy)/underAC)
    A = math.asin(arm1Length*math.sin(B)/(ac))

    B = math.degrees(B)
    O = math.degrees(O)
    A = math.degrees(A)

    a = A-O
    servo0angle = arm1AngleC + (180 - B)
    servo1angle = arm2AngleC - a
    servo2angle = 82 - Zangle

    leg[0].angle = servo0angle
    leg[1].angle = servo1angle
    leg[2].angle = servo2angle

    print("ZAngle:{}".format(Zangle))
    print("toltalArm:{}".format(totalArm3Length))
    print("ac:{}".format(ac))
    print("B:{}".format(B))
    print("O:{}".format(O))
    print("A:{}".format(A))
    print("a:{}".format(a))


for i in legs:
    moveToCartesian(i, 0, 0, 0)