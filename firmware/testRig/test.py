from __future__ import division
import math
import time

def Interpolation():
    EffectPercentage = 1
    Step = 1
    nSteps = 32
    MovementStepPercentage = EffectPercentage/nSteps*Step
    print(MovementStepPercentage)
    while Step < 32:
        Interpolation = (1-math.sin(MovementStepPercentage*180+90))/2*EffectPercentage+MovementStepPercentage*(1-EffectPercentage)
        print ("Interpolation =", Interpolation)
        Step = Step+1
        print ("Step ="), Step
        MovementStepPercentage = EffectPercentage/nSteps*Step
        print (MovementStepPercentage)
        time.sleep(0.0035)
        if Step == 32:
            break

Interpolation()