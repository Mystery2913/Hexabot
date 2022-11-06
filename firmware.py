"""
GNU General Public License v3.0 (GPL-3.0)

This file is part of the Hexabot Project (https://github.com/Mystery2913/Hexabot).
Copyright (C) 2022 Dylan Sharm (Mystery2913).

Copyright (C) 2007 Free Software Foundation, Inc. https://fsf.org/
Everyone is permitted to copy and distribute verbatim copies of this license document, but changing it is not allowed.

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, version 3 of the License. This program is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along
with this program. If not, see <https://www.gnu.org/licenses/>.
"""

import math
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

class Hexabot:
    """
    A class to control Hexabot robot
    ================================
    * Author: Dylan Sharm

    ...

    Args:
        None

    Returns:
        None

    Methods
    -------
    `walkForward`(speed) -> None
        Moves Hexabot forward by 1 step of 3 steps in cycle

    `walkForwardSlow`(speed) -> None
        Moves Hexabot forward by 1 step of 6 steps in cycle
    
    `disableMotors`() -> None
        Disables all motors, will only stop motors from holding position if power is cycled

    """

    def __init__(self) -> None:
        # Initialise busIO by linking to I²C bus lines and initialise the 2 PCA9685 chips with 50Hz PWM frequency.
        self.i2c = busio.I2C(SCL, SDA)
        # pca0 is the first servo driver with the address of 64 (0x40) and pca1 is the second servo driver with the address of 65 (0x41) and has its A0 solder pads bridged.
        self.pca0 = PCA9685(self.i2c, address=64)
        self.pca1 = PCA9685(self.i2c, address=65)
        self.pca0.frequency = 50
        self.pca1.frequency = 50
        # updateRate is a value for the time in seconds it takes to execute an interpolation sequence frame. It can be adjusted here to tune the accuracy of the duration of each movement.
        self.updateRate = 0.1
        self.pcaSet = [self.pca0, self.pca1]
        # Initialise each leg with its corresponding indexes.
        self.leg0 = Leg(0, self.pcaSet, self.updateRate)
        self.leg1 = Leg(1, self.pcaSet, self.updateRate)
        self.leg2 = Leg(2, self.pcaSet, self.updateRate)
        self.leg3 = Leg(3, self.pcaSet, self.updateRate)
        self.leg4 = Leg(4, self.pcaSet, self.updateRate)
        self.leg5 = Leg(5, self.pcaSet, self.updateRate)
        self.legs = [self.leg0, self.leg1, self.leg2, self.leg3, self.leg4, self.leg5]
        # slowWalkingIndexList defines the sequence of the walkForwardSlow() method, and slowWalkingIndexNum stores the leg to be moved.
        self.slowWalkingIndexList = [2, 3, 1, 4, 0, 5]
        self.slowWalkingIndexNum = 0

    def walkForward(self, speed: float) -> None:
        """
        Moves Hexabot forward by 1 step of 3 steps in cycle.
        
        Cycles each leg through 3 coordinate states over a period of time defined by `speed`.
        Each execution acts as one step, and the walking index of each leg is stored in the leg. 

        Args:
            `speed` (float): Defines the speed of the step, is a percentage between 0 - 1.
        
        Returns:
            None
        """
        # Each set of coordinates corresponds to the positions of 3 points in the walking sequence. Index 2 of these lists are angles not Z coords.
        forwardUpCoord = [-20, 10, 15]
        forwardDownCoord = [-80, -120, 15]
        backDownCoord = [-80, -120, -15]
        coords = [forwardUpCoord, forwardDownCoord, backDownCoord]
        # shortestDurationStep specifies the duration of the fastest move and longestDurationStep specifies the duration of the slowest move. Values are in seconds.
        shortestDurationStep = 0.5
        longestDurationStep = 3
        # The `speed` argument is a percentage between 0 and 1, and then is scaled to the shortest and longest duration. E.g. if `speed` was 1, the stepDuration would be 0.5.
        stepDuration = longestDurationStep + (((speed-0)*(shortestDurationStep-longestDurationStep))/(1-0))

        for leg in self.legs:
            # Assign each legs current x and y coords to its own x and y attribute. The position in the sequence of each leg is determined by its walking index.
            leg.x = coords[leg.walkingIndex][0]
            leg.y = coords[leg.walkingIndex][1]
            # The legs that are on the left side of Hexabot (leg index 0-2) need to have their zAngle inverted, since all legs need to be moving in the same direction.
            if math.floor(leg.index/3) == 0:
                leg.zAngle = (coords[leg.walkingIndex][2]) * -1
            else:
                leg.zAngle = (coords[leg.walkingIndex][2])
            # Each legs increment arrays are generated using the assigned coords 
            leg.incrementArray = leg.processLegCoord(leg.x, leg.y, zAngle = leg.zAngle, duration=stepDuration)
        
        # The number of frames in the interpolation sequence is determined by stepDuration / self.updateRate.
        for i in range(int(stepDuration / self.updateRate)):
            # For each interpolation sequence frame move each leg on step along in its sequence. This means that the legs move simultaneously.
            for leg in self.legs:
                leg.moveFrame(leg.incrementArray)

        # After one step shift each legs.
        for leg in self.legs:
            leg.shiftWalkingIndex()
    

    def walkForwardSlow(self, speed: float) -> None:
        """
        Moves Hexabot forward by 1 step of 6 steps in cycle.
        
        Cycles 1 leg at a time through 3 coordinate states over a period of time defined by `speed`.
        Each execution acts as one step, and the current leg this is next leg to move is stored in Hexabot. 

        Args:
            `speed` (float): Defines the speed of the step, is a percentage between 0 - 1.
        
        Returns:
            None
        """
        # Each set of coordinates corresponds to the positions of 3 points in the walking sequence. Index 2 of these lists are angles not Z coords.
        forwardUpCoord = [-20, 10, 15]
        forwardDownCoord = [-80, -120, 15]
        backDownCoord = [-80, -120, -15]
        coords = [forwardUpCoord, forwardDownCoord, backDownCoord]
        # shortestDurationStep specifies the duration of the fastest move and longestDurationStep specifies the duration of the slowest move. Values are in seconds.
        shortestDurationStep = 0.5
        longestDurationStep = 3
        # shortestDurationStep specifies the duration of the fastest pause and longestDurationStep specifies the duration of the slowest pause. Values are in seconds.
        shortestDurationPause = 0.5
        longestDurationPause = 1.5
        
        # The `speed` argument is a percentage between 0 and 1, and then is scaled to the shortest and longest duration. E.g. if `speed` was 1, the stepDuration would be 0.5.
        stepDuration = longestDurationStep + (((speed-0)*(shortestDurationStep-longestDurationStep))/(1-0))
        pauseDuration = (longestDurationPause) + (((speed-0)*((shortestDurationPause)-(longestDurationPause)))/(1-0))

        for i in range(3):
            # Find leg depending on the current slowWalkingIndexNum
            leg = self.legs[self.slowWalkingIndexList[self.slowWalkingIndexNum]]

            # Assign the leg is x, y, and z angle to its object.
            leg.x = coords[i][0]
            leg.y = coords[i][1]
            # The legs that are on the left side of Hexabot (leg index 0-2) need to have their zAngle inverted, since all legs need to be moving in the same direction.
            if math.floor(leg.index/3) == 0:
                leg.zAngle = (coords[i][2]) * -1
            else:
                leg.zAngle = (coords[i][2])
            # Each legs increment arrays are generated using the assigned coords 
            leg.incrementArray = leg.processLegCoord(leg.x, leg.y, zAngle = leg.zAngle, duration=stepDuration)
            
            # The number of frames in the interpolation sequence is determined by stepDuration / self.updateRate.
            for i in range(int(stepDuration / self.updateRate)):
                # For each interpolation sequence frame move each leg on step along in its sequence. This means that the legs move simultaneously.
                leg.moveFrame(leg.incrementArray)
            time.sleep(pauseDuration)
        
        # Increment the sequence list
        self.slowWalkingIndexNum += 1
        if self.slowWalkingIndexNum > 5:
            self.slowWalkingIndexNum = 0


    def disableMotors(self) -> None:
        """
        Disables all motors.
        
        Only stops motors from holding position if power is cycled off and on.

        Args:
            None
        
        Returns:
            None
        """
        # Runs the disable leg method for each leg.
        for leg in self.legs:
            leg.disableLeg()
    
    

class Leg:
    """
    A class to control a leg of the Hexabot robot
    =============================================
    * Author: Dylan Sharm

    ...

    Args:
        `index` (int): Defines which leg of all 6 legs this class controls. In range 0 - 5.
        `pcaArray` (list): Passes the 2 PCA9685 chip objects in an list.
        `updateRate` (float): Passed in value for updateRate, can be adjusted in `Hexabot` class.
    
    Returns:
        None

    Methods
    -------
    `processLegCoord`(targetX, targetY, targetZ = None, duration = None, zAngle=None, updateRate = 0.1) -> list
        Returns an list of the increments for linear interpolation for a coordinate set.
    
    `moveFrame` (steps) -> None
        Moves leg one frame in its interpolation sequence.
    
    `moveLeg` (targetX, targetY, targetZ, duration = None, zAngle=None, updateRate = 0.1) -> None
        Moves leg over all frames in its interpolation sequence, moving it to its interpolation sequences inputted coordinates.

    `shiftWalkingIndex` () -> None:
        Shifts the walking index of the leg 1 step forward in sequence.

    `disableLeg` () -> None
        Disables all 3 motors for leg.
    """
    def __init__(self, index: int, pcaArray: list, updateRate: float) -> None:
        """
        Args:
            `index` (int): Defines which leg of all 6 legs this class controls. In range 0 - 5.
            `pcaArray` (list): Passes the 2 PCA9685 chip objects in an list.
            `updateRate` (float): Passed in value for updateRate from `Hexabot` class
        
        Returns:
            None
        """
        self.index = index
        self.pcaArray = pcaArray
        self.updateRate = updateRate
        # minPulse sets the minimum pulse value for the servo motor and maxPulse sets the maximum pulse value for the servo motor. Changing these values will adjust the accuracy of the 180 degree range of the motors.
        self.minPulse = 650
        self.maxPulse = 2800
        # To initialise the servo motor, the first argument needs to be in the format pcaX.channels[Y]. For legs index 0-2 pca0 as is used, X = 0 (which is why 'math.floor(self.index/3)' is used)
        # For legs index 3-5, pca1 is used, X = 1. Then each motor needs to be assigned to an address which is depended on the index of the leg.
        # '(3 * self.index)' will incrementally assign the motors to the correct address, however, with the midway change of pca objects, the address must be reset (which is why '- (math.floor(self.index/3) * 9)' is used).
        # minPulse and maxPulse are also passed to each motor.
        self.servo0 = servo.Servo(self.pcaArray[math.floor(self.index/3)].channels[(3 * self.index) - (math.floor(self.index/3) * 9)], min_pulse=self.minPulse, max_pulse=self.maxPulse)
        self.servo1 = servo.Servo(self.pcaArray[math.floor(self.index/3)].channels[(3 * self.index + 1) - (math.floor(self.index/3) * 9)], min_pulse=self.minPulse, max_pulse=self.maxPulse)
        self.servo2 = servo.Servo(self.pcaArray[math.floor(self.index/3)].channels[(3 * self.index + 2) - (math.floor(self.index/3) * 9)], min_pulse=self.minPulse, max_pulse=self.maxPulse)
        self.incrementArray = []
        self.x = 0
        self.y = 0
        self.z = 0
        self.zAngle = 0
        # Each leg was its walking index assigned based on the rules below. Importantly, each side is offset to each other such that it is shifted by one step.
        if math.floor(self.index/3) == 0:
            self.walkingIndex = abs(self.index - 2)
        else:
            if self.index == 5:
                self.walkingIndex = 0
            elif self.index == 3:
                self.walkingIndex = 1
            elif self.index == 4:
                self.walkingIndex = 2
        
    
    def processLegCoord (self, targetX: int, targetY: int, targetZ: int = None, duration: float = None, zAngle: int = None) -> list:
        """
        Returns an array of the increments for linear interpolation for a cartesian coordinate system.

        Either a Z coordinate (`targetZ`) or a relative Z angle (`zAngle`) can be used for output position. 
        Coordinates are used to calculate absolute angle positions for each motor in the leg, then they are interpolated over a `duration`.
        Output is an array containing the increments for each motor's angle per frame of the interpolation sequence.
        The length of the interpolation sequence is determined by the `duration` in seconds of the movement and `updateRate` in seconds, which is the time for the code segment to execute.
        `updateRate` is a `Hexabot` class attribute that can be changed to tune the accuracy of the duration period of a movement.

        Args:
            `targetX` (int): X cartesian coordinate target.
            `targetY` (int): Y cartesian coordinate target.
            `targetZ` (int): Z cartesian coordinate target | (default is None)
            `duration` (float): Duration of interpolation sequence | (default is None)
            `zAngle` (int): Relative angle of Z axis motor, can be used instead of a target Z coordinate | (default is None)

        
        Returns:
            list: a list contain each motor's increment per frame of its interpolation sequence.
        """
        # These are constants that are defined by the geometry of Hexabot. These should not be changed unless changes to the geometry have been made.
        arm1AngleC = 43
        arm2AngleC = 150
        arm2PointY = 57.939
        arm3PointX = -215.517
        arm2To3X = 98.50
        arm1Length = 140
        arm2Length = 90

        # If a value for zAngle has been passed, then zAngle is directed used for the angle and default value for totalArm3Length are used.
        if not zAngle == None:
            totalArm3Length = math.sqrt((abs(arm3PointX)+targetX)**2+(0)**2)
            underAC = totalArm3Length - arm2To3X
            Zangle = zAngle
        # Otherwise if targetZ is used then zAngle is calculated using inverse kinematics.
        else:
            Zangle = math.atan(targetZ/(abs(arm3PointX)+targetX))
            totalArm3Length = math.sqrt((abs(arm3PointX)+targetX)**2+(targetZ)**2)
            underAC = totalArm3Length - arm2To3X
            Zangle = math.degrees(Zangle)
        
        # See supporting documenting for inverse kinematics model used for angle calculations (**Link to supporting documenting**).
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

        # If motors are disabled, current values need to be reset.
        if self.servo0.angle == None or self.servo1.angle == None or self.servo2.angle == None:
            self.servo0.angle = 160
            self.servo1.angle = 110
            self.servo2.angle = 90

        # When booted, the values for the servo angles can be larger than 180, this resets this value.
        if self.servo0.angle > 180 or self.servo1.angle > 180 or self.servo2.angle > 180:
            self.servo0.angle = 160
            self.servo1.angle = 110
            self.servo2.angle = 90

        # Calculate the difference between the absolute current angle and the absolute target angle.
        servo0Dif = servo0angle - self.servo0.angle
        servo1Dif = servo1angle - self.servo1.angle
        servo2Dif = servo2angle - self.servo2.angle
        # Calculate the incremental value for the interpolation sequence based on the duration and updateRate.
        servoIncrement0 = servo0Dif / (duration / self.updateRate)
        servoIncrement1 = servo1Dif / (duration / self.updateRate)
        servoIncrement2 = servo2Dif / (duration / self.updateRate)

        # Return incremental values in list.
        return [servoIncrement0, servoIncrement1, servoIncrement2]
    
    def moveFrame (self, steps: list) -> None:
        """
        Moves leg one frame in its interpolation sequence.

        Can be executed with each legs current frame, so all legs move in sync.

        Args:
            `steps` (list): List contain each motor's increment per frame of its interpolation sequence.
        
        Returns:
            None
        """
        # Add the passed in incremental steps to the current angle.
        self.servo0.angle = self.servo0.angle + steps[0]
        self.servo1.angle = self.servo1.angle + steps[1]
        self.servo2.angle = self.servo2.angle + steps[2]

    def moveLeg(self, targetX: int, targetY: int, targetZ: int = None, duration: float = None, zAngle: int = None) -> None:
        """
        Moves leg over all frames in its interpolation sequence, moving it to its interpolation sequences inputted coordinates.
        
        Moves leg to its increments inputted coordinates directly, meaning it cannot be used to move legs in sync.

        Args:
            `targetX` (int): X cartesian coordinate target.
            `targetY` (int): Y cartesian coordinate target.
            `targetZ` (int): Z cartesian coordinate target | (default is None)
            `duration` (float): Duration of interpolation sequence | (default is None)
            `zAngle` (int): Relative angle of Z axis motor, can be used instead of a target Z coordinate | (default is None)
        
        Returns:
            None
        """
        # Get the increments for argument coords.
        increments = self.processLegCoord(targetX, targetY, targetZ, duration, zAngle)

        # Loop through all frames of interpolation sequence.
        for i in range(int(duration / self.updateRate)):
            self.moveFrame(increments)
    
    def shiftWalkingIndex (self) -> None:
        """
        Shifts the walking index of the leg 1 step forward in sequence.
        
        Loops back on itself from 2 -> 0

        Args:
            None
        
        Returns:
            None
        """
        # Shift the walking index to the right, 3 loops back to 0.
        self.walkingIndex = self.walkingIndex + 1
        if self.walkingIndex > 2:
            self.walkingIndex = 0
    
    def disableLeg(self) -> None:
        """
        Disables all 3 motors for leg.

        Args:
            None
        
        Returns:
            None
        """
        # Setting the servo angle to None disables the angle assigned to the motors, since otherwise they will hold position.
        self.servo0.angle = None
        self.servo1.angle = None
        self.servo2.angle = None

class stackExecute():
    """
    A class to control a leg of the Hexabot robot.
    =============================================
    * Author: Dylan Sharm

    ...

    Args:
        None
    
    Returns:
        None

    Methods
    -------
    `queue` (walkingState, speed) -> None
        Checks if the state of walking is True, then executes `Hexabot.walkForward()`.
    """
    def __init__(self) -> None:
        self.robot = Hexabot()


    def queue(self, walkingStateSlow: object, walkingState: object, speed: object) -> None:
        """
        Checks if the state of walking is True, then executes `Hexabot.walkForward()`.

        Is run in another process using multiprocessing and passed the shared memory variable objects: `walkingState` and `speed`.

        Args:
            `walkingState` (object): Shared memory variable object contained a bool value for the state of the walking command.
            `speed` (object): Shared memory variable object contained a float of the speed that is passed to the `Hexabot.walkForward()` method, is a percentage between 0 - 1.
        
        Returns:
            None
        """
        while True:
            # Check weather or not the value of `walkingState` or `walkingStateSlow` has been updated. If so, call the Hexabot.walkForward() or the Hexabot.walkForwardSlow() methods and pass it the speed value from `speed`.
            if walkingState.value == True:
                robot.walkForward(speed.value)
                walkingState.value = False
            if walkingStateSlow.value == True:
                robot.walkForwardSlow(speed.value)
                walkingStateSlow.value = False


robot = Hexabot()

# When app.py is run, this will make the robot start up with its legs in the default positions.
for i in robot.legs:
    i.moveLeg(-80,-120, 0, 0.5)