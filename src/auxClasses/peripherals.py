import time
import queue
import busio
from gpiozero import Robot, Motor, Servo, DistanceSensor, PWMOutputDevice, RotaryEncoder, DigitalInputDevice
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1115 as ADS
import adafruit_icm20x as IMU
from auxClasses.infraredSensorMngr import InfraredSensorMngr
from enum import Enum

# Enum for forward and backward directions
class Direction(Enum):
   BACKWARD = 0
   FORWARD = 1

MOTOR_STEPS_PER_TURN = 872
ROBOT_RADIUS_FROM_CENTER_IN_M = 0.1
WHEEL_DIAMETER_IN_M = 0.068
PWM_FOR_MAX_SPEED = 0.8
PWM_FOR_MIN_SPEED = 0.2
MAX_MOTOR_RMP = 100
ROBOT_PWM_RAMP_RATE = 0.02
MAX_SPEED_IN_STEPS_PER_S = (MAX_MOTOR_RMP * MOTOR_STEPS_PER_TURN) / 60
WHEEL_CIRCUMFERENCE_IN_M = 3.141592 * WHEEL_DIAMETER_IN_M
MOTOR_STEPS_PER_M = MOTOR_STEPS_PER_TURN / WHEEL_CIRCUMFERENCE_IN_M

# A class that contains all the peripherals of the robot
# It offers all the necessary functions to interact with the peripherals, such as reading sensors,
# controlling motors, etc.
class Peripherals:
   def __init__(self):
      self.vacuumMotor      = PWMOutputDevice(pin=12)
      self.leftMotor        = Motor(26, 19)
      self.rightMotor       = Motor(21, 20)
      self.encoderLeft      = RotaryEncoder(a=5, b=6, max_steps=0) # 872 steps/turn
      self.encoderRight     = RotaryEncoder(a=24, b=25, max_steps=0) # 872 steps/turn
      self.leftDistSens     = DistanceSensor(echo=27, trigger=17, threshold_distance=0.15)
      # self.frontDistSens   = DistanceSensor(echo=22, trigger=11, threshold_distance=0.15)
      # self.rightDistSens   = DistanceSensor(echo=10, trigger=0, threshold_distance=0.15)
      # self.backDistSens    = DistanceSensor(echo=9, trigger=13, threshold_distance=0.15)
      self.tubeSensMngr     = InfraredSensorMngr(frontPin=14, backPin=15)
      self.hallEffectSens   = DigitalInputDevice(pin=23, pull_up=None, active_state=False)
      self.servo            = Servo(pin=16)
      self.ads              = ADS.ADS1115(busio.I2C(scl=3, sda=2))
      self.chargerCnnctd    = AnalogIn(self.ads, ADS.P0)
      self.vacuumBattery    = AnalogIn(self.ads, ADS.P2)
      self.elctrnicsBattery = AnalogIn(self.ads, ADS.P1)
      # self.imu            = IMU.ICM20948(busio.I2C(scl=3, sda=2))
      self.leftMotorTargetStepsPerS:  float = 0
      self.rightMotorTargetStepsPerS: float = 0
      self.currLeftMotorPWM:  float = 0
      self.currRightMotorPWM: float = 0
      self.lastLeftMotorSteps:  int = 0
      self.lastRightMotorSteps: int = 0
      self.lastStepsReadTime: float = 0
      self.robotDirection = Direction.FORWARD # 1 for forward, 0 for backward

   def close(self):
      self.leftMotor.stop()
      self.rightMotor.stop()
      self.vacuumMotor.off()
      self.encoderLeft.close()
      self.encoderRight.close()
      self.leftDistSens.close()
      # self.frontDistSens.close()
      # self.rightDistSens.close()
      # self.backDistSens.close()
      self.servo.detach()
      self.tubeSensMngr.close()

   def safeExit(self, signum, frame):
      self.leftMotor.stop()
      self.rightMotor.stop()
      exit(1)

   def getLeftEncoderSteps(self) -> int:
      return self.encoderLeft.steps
   def getRightEncoderSteps(self) -> int:
      return self.encoderRight.steps
   def resetEncoders(self):
      self.encoderLeft.steps  = 0
      self.encoderRight.steps = 0

   def getLeftMotorTargetStepsPerS(self) -> float:
      return self.leftMotorTargetStepsPerS
   def getRightMotorTargetStepsPerS(self) -> float:
      return self.rightMotorTargetStepsPerS
   
   def getLeftDistance(self) -> float:
      return self.leftDistSens.distance * 100.0
   # def getFrontDistance(self) -> float:
   #    return self.frontDistSens.distance * 100.0
   # def getRightDistance(self) -> float:
   #    return self.rightDistSens.distance * 100.0
   # def getBackDistance(self) -> float:
   #    return self.backDistSens.distance * 100.0

   def setVacuumMotorPWM(self, pwm: float):
      if (pwm >= 0) and (pwm <= 1):
         self.vacuumMotor.blink(on_time=(0.01*pwm), off_time=(0.01*(1-pwm)))

   # Changes the motors PWM according to the read encoder values and the target values
   def controlMotorsPWM(self):
      currTime = time.time()
      timeDiff = currTime - self.lastStepsReadTime

      if timeDiff < 2.0:
         return

      self.lastStepsReadTime = currTime

      leftMotorStepsDiff  = abs(self.encoderLeft.steps - self.lastLeftMotorSteps)
      rightMotorStepsDiff = abs(self.encoderRight.steps - self.lastRightMotorSteps)

      self.lastLeftMotorSteps  = self.encoderLeft.steps
      self.lastRightMotorSteps = self.encoderRight.steps

      leftMotorStepsPerS  = (leftMotorStepsDiff / timeDiff)
      rightMotorStepsPerS = (rightMotorStepsDiff / timeDiff)

      leftMotorError  = self.leftMotorTargetStepsPerS - leftMotorStepsPerS
      rightMotorError = self.rightMotorTargetStepsPerS - rightMotorStepsPerS

      leftMotorPWM  = self.currLeftMotorPWM  + (leftMotorError  * ROBOT_PWM_RAMP_RATE)
      rightMotorPWM = self.currRightMotorPWM + (rightMotorError * ROBOT_PWM_RAMP_RATE)

      if (leftMotorPWM < 0.0):
         leftMotorPWM = 0.0
      elif (leftMotorPWM > 1.0):
         leftMotorPWM = 1.0
      if (rightMotorPWM < 0.0):
         rightMotorPWM = 0.0
      elif (rightMotorPWM > 1.0):
         rightMotorPWM = 1.0

      self.currLeftMotorPWM  = leftMotorPWM
      self.currRightMotorPWM = rightMotorPWM

      if leftMotorPWM > 0:
         if self.robotDirection == Direction.FORWARD:
            self.leftMotor.forward(speed=leftMotorPWM)
         else:
            self.leftMotor.backward(speed=leftMotorPWM)
      else:
         self.leftMotor.stop()
      if rightMotorPWM > 0:
         if self.robotDirection == Direction.FORWARD:
            self.rightMotor.forward(speed=rightMotorPWM)
         else:
            self.rightMotor.backward(speed=rightMotorPWM)
      else:
         self.rightMotor.stop()

   # Sets the target speed for the robot to move forward
   # Left and right relative to the robot itself
   def driveRobotForward(self, speed: float, curveLeftRate: float, curveRightRate: float):
      if ((curveLeftRate < 0 or curveLeftRate > 1) or
          (curveRightRate < 0 or curveRightRate > 1) or
          (speed < 0 or speed > 1)):
         print("Invalid speed values")
         return
      
      targetStepsPerS = 0.0
      if (speed < PWM_FOR_MIN_SPEED):
         targetStepsPerS = 0.0
      elif (speed > PWM_FOR_MAX_SPEED):
         targetStepsPerS = MAX_SPEED_IN_STEPS_PER_S
      else:
         targetStepsPerS = ((speed - PWM_FOR_MIN_SPEED) / (PWM_FOR_MAX_SPEED - PWM_FOR_MIN_SPEED)) * MAX_SPEED_IN_STEPS_PER_S

      if (curveLeftRate == 0 and curveRightRate == 0):
         self.leftMotorTargetStepsPerS  = targetStepsPerS
         self.rightMotorTargetStepsPerS = targetStepsPerS
      elif (curveLeftRate == 0 and curveRightRate != 0):
         self.leftMotorTargetStepsPerS  = targetStepsPerS
         self.rightMotorTargetStepsPerS = targetStepsPerS * (1 - curveRightRate)
      elif (curveLeftRate != 0 and curveRightRate == 0):
         self.leftMotorTargetStepsPerS  = targetStepsPerS * (1 - curveLeftRate)
         self.rightMotorTargetStepsPerS = targetStepsPerS
      else: # curveLeftRate and curveRightRate are exclusive compared to each other
         self.leftMotorTargetStepsPerS  = 0
         self.rightMotorTargetStepsPerS = 0
   
      self.robotDirection = Direction.FORWARD

   # Sets the target speed for the robot to move backward
   # Left and right relative to the robot itself
   def driveRobotBackward(self, speed: float, curveLeftRate: float, curveRightRate: float):
      if ((curveLeftRate < 0 or curveLeftRate > 1) or
          (curveRightRate < 0 or curveRightRate > 1) or
          (speed < 0 or speed > 1)):
         print("Invalid speed values")
         return
      
      targetStepsPerS = 0
      if (speed < PWM_FOR_MIN_SPEED):
         targetStepsPerS = 0
      elif (speed > PWM_FOR_MAX_SPEED):
         targetStepsPerS = MAX_SPEED_IN_STEPS_PER_S
      else:
         targetStepsPerS = ((speed - PWM_FOR_MIN_SPEED) / (PWM_FOR_MAX_SPEED - PWM_FOR_MIN_SPEED)) * MAX_SPEED_IN_STEPS_PER_S

      if (curveLeftRate == 0 and curveRightRate == 0):
         self.leftMotorTargetStepsPerS  = targetStepsPerS
         self.rightMotorTargetStepsPerS = targetStepsPerS
      elif (curveLeftRate == 0 and curveRightRate != 0):
         self.leftMotorTargetStepsPerS  = targetStepsPerS
         self.rightMotorTargetStepsPerS = targetStepsPerS * (1 - curveRightRate)
      elif (curveLeftRate != 0 and curveRightRate == 0):
         self.leftMotorTargetStepsPerS  = targetStepsPerS * (1 - curveLeftRate)
         self.rightMotorTargetStepsPerS = targetStepsPerS
      else: # curveLeftRate and curveRightRate are exclusive compared to each other
         self.leftMotorTargetStepsPerS  = 0
         self.rightMotorTargetStepsPerS = 0

      self.robotDirection = Direction.BACKWARD

   # Rotates the robot in its own axis, given an direction and an agle in radians
   # direction: 0 for left, 1 for right
   # angle: angle in radians
   def rotate(self, direction: int, angle: float, speed: float):
      initialSteps: int = 0
      encoderSteps = {0: self.encoderRight.steps, 1: self.encoderLeft.steps}

      rebasedSpeed = (speed * (PWM_FOR_MAX_SPEED - PWM_FOR_MIN_SPEED)) + PWM_FOR_MIN_SPEED

      if (direction == 0):
         initialSteps = self.encoderRight.steps
         self.rightMotor.forward(speed=rebasedSpeed)
         self.leftMotor.backward(speed=rebasedSpeed)
      elif (direction == 1):
         initialSteps = self.encoderLeft.steps
         self.leftMotor.forward(speed=rebasedSpeed)
         self.rightMotor.backward(speed=rebasedSpeed)
      else:
         print("Invalid direction")
         return
      
      archSize = angle * ROBOT_RADIUS_FROM_CENTER_IN_M
      stepsToTurn = archSize * MOTOR_STEPS_PER_M

      while (abs(encoderSteps[direction] - initialSteps) < stepsToTurn):
         time.sleep(0.01)
      
      self.leftMotor.stop()
      self.rightMotor.stop()

      return
   
   def stopRobot(self):
      self.leftMotor.stop()
      self.rightMotor.stop()

   def isHallEffectSensActive(self) -> bool:
      return self.hallEffectSens.is_active
   
   def getLowerBatteryLvl(self) -> float:
      minVoltage = min(self.vacuumBattery.voltage, self.elctrnicsBattery.voltage)
      minVoltage = (minVoltage / 3.3) * 100
      return minVoltage
   def getLowerBatteryADCVal(self) -> int:
      minADCVal = min(self.vacuumBattery.value, self.elctrnicsBattery.value)
      return minADCVal
   
   def isChargerConnected(self) -> bool:
      return (self.chargerCnnctd.voltage > 3.0)
   
   def getCollectedBallsQty(self) -> int:
      return self.tubeSensMngr.ballCount
   def isBallStuck(self) -> bool:
      return self.tubeSensMngr.isBallStuck()