import time
import queue
import busio
from gpiozero import Robot, Motor, Servo, DistanceSensor, PWMOutputDevice, RotaryEncoder, DigitalInputDevice
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1115 as ADS
import adafruit_icm20x as IMU
from auxClasses.infraredSensorMngr import InfraredSensorMngr

MOTOR_STEPS_PER_TURN = 872
ROBOT_RADIUS_FROM_CENTER_IN_M = 0.1
WHEEL_DIAMETER_IN_M = 0.068
WHEEL_CIRCUMFERENCE_IN_M = 3.141592 * WHEEL_DIAMETER_IN_M
MOTOR_STEPS_PER_M = MOTOR_STEPS_PER_TURN / WHEEL_CIRCUMFERENCE_IN_M

# A class that contains all the peripherals of the robot
# It offers all the necessary functions to interact with the peripherals, such as reading sensors,
# controlling motors, etc.
class Peripherals:
   def __init__(self):
      self.vacuumMotor      = PWMOutputDevice(pin=12)
      self.robot            = Robot(left=(Motor(26, 19)), right=(Motor(21, 20)))
      self.encoderLeft      = RotaryEncoder(a=5, b=6, max_steps=0) # 872 steps/turn
      self.encoderRight     = RotaryEncoder(a=25, b=16, max_steps=0) # 872 steps/turn
      self.leftDistSens     = DistanceSensor(echo=27, trigger=17, threshold_distance=0.15)
      # self.frontDistSens   = DistanceSensor(echo=22, trigger=17, threshold_distance=0.15)
      # self.rightDistSens   = DistanceSensor(echo=10, trigger=17, threshold_distance=0.15)
      # self.backDistSens    = DistanceSensor(echo=9, trigger=17, threshold_distance=0.15)
      self.tubeSensMngr     = InfraredSensorMngr(frontPin=14, backPin=15)
      self.hallEffectSens   = DigitalInputDevice(pin=23, pull_up=None, active_state=False)
      self.servo            = Servo(pin=24)
      self.ads              = ADS.ADS1115(busio.I2C(scl=3, sda=2))
      self.chargerCnnctd    = AnalogIn(self.ads, ADS.P0)
      self.vacuumBattery    = AnalogIn(self.ads, ADS.P2)
      self.elctrnicsBattery = AnalogIn(self.ads, ADS.P1)
      # self.imu            = IMU.ICM20948(busio.I2C(scl=3, sda=2))

   def close(self):
      self.robot.stop()
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
      self.robot.stop()
      exit(1)

   def getLeftEncoderSteps(self) -> int:
      return self.encoderLeft.steps
   def getRightEncoderSteps(self) -> int:
      return self.encoderRight.steps
   def resetEncoders(self):
      self.encoderLeft.steps  = 0
      self.encoderRight.steps = 0
   
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

   # Drive the robot forward by running both motors forward.
   # Left and right relative to the robot itself
   def driveRobotForward(self, speed: float, curveLeftRate: float, curveRightRate: float):
      if ((curveLeftRate < 0 or curveLeftRate > 1) or
          (curveRightRate < 0 or curveRightRate > 1) or
          (speed < 0 or speed > 1)):
         print("Invalid speed values")
         return
      
      self.robot.forward(speed=speed, curve_left=curveLeftRate, curve_right=curveRightRate)
   
   # Drive the robot backward by running both motors backward.
   # Left and right relative to the robot itself
   def driveRobotBackward(self, speed: float, curveLeftRate: float, curveRightRate: float):
      if ((curveLeftRate < 0 or curveLeftRate > 1) or
          (curveRightRate < 0 or curveRightRate > 1) or
          (speed < 0 or speed > 1)):
         print("Invalid speed values")
         return
      
      self.robot.backward(speed=speed, curve_left=curveLeftRate, curve_right=curveRightRate)

   # Rotates the robot in its own axis, given an direction and an agle in radians
   # direction: 0 for left, 1 for right
   # angle: angle in radians
   def rotate(self, direction: int, angle: float):
      initialSteps: int = 0
      encoderSteps = {0: self.encoderRight.steps, 1: self.encoderLeft.steps}

      if (direction == 0):
         initialSteps = self.encoderRight.steps
         self.robot.left()
      elif (direction == 1):
         initialSteps = self.encoderLeft.steps
         self.robot.right()
      else:
         print("Invalid direction")
         return
      
      archSize = angle * ROBOT_RADIUS_FROM_CENTER_IN_M
      stepsToTurn = archSize * MOTOR_STEPS_PER_M

      while (abs(encoderSteps[direction] - initialSteps) < stepsToTurn):
         time.sleep(0.01)
      
      self.robot.stop()

      return
   
   def stopRobot(self):
      self.robot.stop()

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