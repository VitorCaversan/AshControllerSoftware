import time
import math

MOTOR_STEPS_PER_TURN = 872
MAX_MOTOR_RMP = 40
MAX_SPEED_IN_STEPS_PER_S = (MAX_MOTOR_RMP * MOTOR_STEPS_PER_TURN) / 60

class StuckDetector:
   def __init__(self) -> None:

      self.stuckTimer = time.time()
      self.prevLeftMotorSteps  = 0
      self.prevRightMotorSteps = 0
      self.isStuck = False

   def routine(self, encoderLeft, encoderRight, pwmLeft: float, pwmRight: float):
      targetStepsPerSLeft  = pwmLeft  * MAX_SPEED_IN_STEPS_PER_S
      targetStepsPerSRight = pwmRight * MAX_SPEED_IN_STEPS_PER_S

      # print (f"stuck timer: {self.stuckTimer}, curr time: {time.time()}")

      if (time.time() - self.stuckTimer) > 1.5:
         self.stuckTimer = time.time()

         if (self.prevLeftMotorSteps != 0 and self.prevRightMotorSteps != 0):
            if (abs(encoderLeft.steps - self.prevLeftMotorSteps) < targetStepsPerSLeft) or (abs(encoderRight.steps - self.prevRightMotorSteps) < targetStepsPerSRight):
                  self.isStuck = True
            else:
                  self.isStuck = False

         print (f"left motor steps: {encoderLeft.steps}, prev left steps: {self.prevLeftMotorSteps}, taget steps/s: {targetStepsPerSLeft}")
         print (f"Right motor steps: {encoderRight.steps}, prev Right steps: {self.prevRightMotorSteps}, taget steps/s: {targetStepsPerSRight}")

         self.prevLeftMotorSteps  = encoderLeft.steps
         self.prevRightMotorSteps = encoderRight.steps
      
   def isRobotStuck(self):
      return self.isStuck