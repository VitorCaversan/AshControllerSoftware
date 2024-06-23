import time
import math

MOTOR_STEPS_PER_TURN = 872
MAX_MOTOR_RMP = 100
MAX_SPEED_IN_STEPS_PER_S = (MAX_MOTOR_RMP * MOTOR_STEPS_PER_TURN) / 60

class StuckDetector:
   def __init__(self) -> None:

      self.stuckTimer = time.time()
      self.stuckLeftMotorSteps  = 0
      self.stuckRightMotorSteps = 0
      self.isStuck = False

   def routine(self, encoderLeft, encoderRight, pwmLeft: float, pwmRight: float):
      targetStepsPerSLeft  = pwmLeft  * MAX_SPEED_IN_STEPS_PER_S
      targetStepsPerSRight = pwmRight * MAX_SPEED_IN_STEPS_PER_S

      if self.stuckTimer - time.time() > 1.5:
         self.stuckTimer = time.time()

         if (abs(encoderLeft.steps - self.stuckLeftMotorSteps) < targetStepsPerSLeft) or (abs(encoderRight.steps - self.stuckRightMotorSteps) < targetStepsPerSRight):
               self.isStuck = True
         else:
               self.isStuck = False

         self.stuckLeftMotorSteps  = encoderLeft.steps
         self.stuckRightMotorSteps = encoderRight.steps
      
   def isRobotStuck(self):
      return self.isStuck