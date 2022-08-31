import math
from pickle import NONE
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from threading import Thread

class Hexabot:
    """Test hello"""

    def __init__(self) -> None:
        self.i2c = busio.I2C(SCL, SDA)
        self.pca0 = PCA9685(self.i2c, address=64)
        self.pca1 = PCA9685(self.i2c, address=65)
        self.pca0.frequency = 50
        self.pca1.frequency = 50
        self.updateRate = 0.1
        self.pcaSet = [self.pca0, self.pca1]
        self.leg0 = Leg(0, self.pcaSet)
        self.leg1 = Leg(1, self.pcaSet)
        self.leg2 = Leg(2, self.pcaSet)
        self.leg3 = Leg(3, self.pcaSet)
        self.leg4 = Leg(4, self.pcaSet)
        self.leg5 = Leg(5, self.pcaSet)
        self.legs = [self.leg0, self.leg1, self.leg2, self.leg3, self.leg4, self.leg5]
    
    def walkForward(self, speed) -> None:
        forwardUpCoord = [-20, 10, 15]
        forwardDownCoord = [-80, -120, 15]
        backDownCoord = [-80, -120, -15]
        coords = [forwardUpCoord, forwardDownCoord, backDownCoord]
        shortestDurration = 0.5
        longestDurration = 3
        stepDuration = longestDurration + (((speed-0)*(shortestDurration-longestDurration))/(1-0))

        for leg in self.legs:
            leg.x = coords[leg.walkingIndex][0]
            leg.y = coords[leg.walkingIndex][1]
            if math.floor(leg.index/3) == 0:
                leg.zAngle = (coords[leg.walkingIndex][2]) * -1
            else:
                leg.zAngle = (coords[leg.walkingIndex][2])
            leg.incrementArray = leg.processLegCoord(leg.x, leg.y, zAngle = leg.zAngle, duration=stepDuration)
        
        updateRateArray = []
        for i in range(int(stepDuration / self.updateRate)):
            timeStart = time.time()
            for leg in self.legs:
                leg.moveFrame(leg.incrementArray)
            updateRateArray.append(time.time()-timeStart)
            print(time.time()-timeStart)
        # self.updateRate = (sum(updateRateArray) / len(updateRateArray))

        for leg in self.legs:
            leg.shiftWalkingIndex()


    def disableMotors(self) -> None:
        self.leg0.disableLeg()
        self.leg1.disableLeg()
        self.leg2.disableLeg()
        self.leg3.disableLeg()
        self.leg4.disableLeg()
        self.leg5.disableLeg()
    
    

class Leg:
    def __init__(self, index, pcaArray) -> None:
        self.index = index
        self.pcaArray = pcaArray
        self.minPulse = 650
        self.maxPulse = 2800
        self.servo0 = servo.Servo(self.pcaArray[math.floor(self.index/3)].channels[(3 * self.index) - (math.floor(self.index/3) * 9)], min_pulse=self.minPulse, max_pulse=self.maxPulse)
        self.servo1 = servo.Servo(self.pcaArray[math.floor(self.index/3)].channels[(3 * self.index + 1) - (math.floor(self.index/3) * 9)], min_pulse=self.minPulse, max_pulse=self.maxPulse)
        self.servo2 = servo.Servo(self.pcaArray[math.floor(self.index/3)].channels[(3 * self.index + 2) - (math.floor(self.index/3) * 9)], min_pulse=self.minPulse, max_pulse=self.maxPulse)
        self.incrementArray = []
        self.x = 0
        self.y = 0
        self.z = 0
        self.zAngle = 0
        if math.floor(self.index/3) == 0:
            self.walkingIndex = abs(self.index - 2)
        else:
            if self.index == 5:
                self.walkingIndex = 0
            elif self.index == 3:
                self.walkingIndex = 1
            else:
                self.walkingIndex = 2
        
    
    def processLegCoord (self, targetX, targetY, targetZ = None, duration = None, zAngle=None, updateRate = 0.1) -> list:

        arm1AngleC = 43
        arm2AngleC = 150
        arm2PointY = 57.939
        arm3PointX = -215.517
        arm2To3X = 98.50
        arm1Length = 140
        arm2Length = 90

        if not zAngle == None:
            totalArm3Length = math.sqrt((abs(arm3PointX)+targetX)**2+(0)**2)
            underAC = totalArm3Length - arm2To3X
            Zangle = zAngle
        else:
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
        servo2angle = 90 - Zangle

        if self.servo0.angle == None or self.servo1.angle == None or self.servo2.angle == None:
            self.servo0.angle = 160
            self.servo1.angle = 110
            self.servo2.angle = 90
        
        if self.servo0.angle > 180 or self.servo1.angle > 180 or self.servo2.angle > 180:
            self.servo0.angle = 160
            self.servo1.angle = 110
            self.servo2.angle = 90


        servo0Dif = servo0angle - self.servo0.angle
        servo1Dif = servo1angle - self.servo1.angle
        servo2Dif = servo2angle - self.servo2.angle
        servoIncrement0 = servo0Dif / (duration / updateRate)
        servoIncrement1 = servo1Dif / (duration / updateRate)
        servoIncrement2 = servo2Dif / (duration / updateRate)

        return [servoIncrement0, servoIncrement1, servoIncrement2]
    
    def moveFrame (self, steps) -> None:
        self.servo0.angle = self.servo0.angle + steps[0]
        self.servo1.angle = self.servo1.angle + steps[1]
        self.servo2.angle = self.servo2.angle + steps[2]

    def moveLeg(self, targetX, targetY, targetZ, duration = None, zAngle=None, updateRate = 0.1) -> None:
        
        increments = self.processLegCoord(targetX, targetY, targetZ, duration, zAngle, updateRate=updateRate)

        for i in range(int(duration / updateRate)):
            self.moveFrame(increments)
            #time.sleep(self.updateRate)
    
    def shiftWalkingIndex (self) -> None:
        self.walkingIndex = self.walkingIndex + 1
        if self.walkingIndex > 2:
            self.walkingIndex = 0
    
    
        
    def disableLeg(self):
        self.servo0.angle = None
        self.servo1.angle = None
        self.servo2.angle = None

class stackExecute():
    def __init__(self) -> None:
        self.robot = Hexabot()


    def queue(self, var, speed):
        while True:
            if var.value == True:
                robot.walkForward(0.8)
                print('lopp')
                var.value = False
    





robot = Hexabot()

# for i in robot.legs:
    

for i in robot.legs:
    i.moveLeg(-80,-120, 0, 0.5)