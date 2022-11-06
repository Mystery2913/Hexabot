from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# Initialise busIO by linking to I²C bus lines and initialise the PCA9685 chip with 50Hz PWM frequency.
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

# Initialise the servo motors with default settings
servo0 = servo.Servo(pca.channels[0], min_pulse=650, max_pulse=2800)
servo1 = servo.Servo(pca.channels[1], min_pulse=650, max_pulse=280)
servo2 = servo.Servo(pca.channels[2], min_pulse=650, max_pulse=280)

# These are the default angles for the positions specified in the Leg Calibration
servo0.angle = 160
servo1.angle = 110
servo2.angle = 90