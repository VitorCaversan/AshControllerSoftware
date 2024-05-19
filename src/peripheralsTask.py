import threading
import time
import queue
import busio
import json
from gpiozero import Robot, Motor, Servo, DistanceSensor, PWMOutputDevice, RotaryEncoder, DigitalInputDevice
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1115 as ADS
import adafruit_icm20x as IMU
from auxClasses.infraredSensorMngr import InfraredSensorMngr

class PeripheralsTask:
   def __init__(self, queue: queue.Queue):
      self.name = "PeripheralsTask"
      self.description = "PeripheralsTask"
      self.mainMsgQueue   = queue
      self.vacuumMotor    = PWMOutputDevice(pin=12)
      self.robot          = Robot(left=(Motor(19, 26)), right=(Motor(20, 21)))
      self.encoderLeft    = RotaryEncoder(a=5, b=6, max_steps=0) # 872 steps/turn
      self.encoderRight   = RotaryEncoder(a=25, b=16, max_steps=0) # 872 steps/turn
      self.leftDistSens   = DistanceSensor(echo=27, trigger=17, threshold_distance=0.15)
      # self.frontDistSens = DistanceSensor(echo=22, trigger=17, threshold_distance=0.15)
      # self.rightDistSens = DistanceSensor(echo=10, trigger=17, threshold_distance=0.15)
      # self.backDistSens  = DistanceSensor(echo=9, trigger=17, threshold_distance=0.15)
      self.tubeSensMngr   = InfraredSensorMngr(frontPin=14, backPin=15)
      self.hallEffectSens = DigitalInputDevice(pin=23, pull_up=None, active_state=False)
      self.servo          = Servo(pin=24)
      self.ads            = ADS.ADS1115(busio.I2C(scl=3, sda=2))
      self.adsChannel     = AnalogIn(self.ads, ADS.P0)
      # self.imu            = IMU.ICM20948(busio.I2C(scl=3, sda=2))
      self.periodicMsg    = {
         "sens_dist_left": 0.0,
         "sens_dist_front": 0.0,
         "sens_dist_right": 0.0,
         "sens_dist_back": 0.0,
         "battery_level": 0.0,
         "balls_collected": 0,
         "balls_coordinates": [
            { "X": 0.0, "Y": 0.0 },
            { "X": 0.0, "Y": 0.0 },
         ],
         "robot_status": "collecting_balls", # Options: collecting_balls, searching_for_balls, returning_to_base, paused
         "robot_error": "base_not_found" # Options: base_not_found, robot_stuck, ball_stuck
      }
      self.thread = threading.Thread(target=self.run)

   def runRobot(self):
      print(f"Sensor distance: {str(self.leftDistSens.distance)}, \nsteps: {str(self.encoderLeft.steps)}, \ncollected balls: {str(self.tubeSensMngr.ballCount)}")
      print(f"Ads value: {str(self.adsChannel.value)}, voltage: {str(self.adsChannel.voltage)}\n")

      if self.hallEffectSens.is_active:
         self.robot.stop()
         self.setVacuumMotorPWM(0.0)
         self.resetEncoders()
         return

      adsCtrlRate = self.adsChannel.voltage / 3.3
      if adsCtrlRate < 0.0:
         adsCtrlRate = 0.0
      elif adsCtrlRate > 1.0:
         adsCtrlRate = 1.0

      if self.leftDistSens.distance < 0.10:
         self.robot.backward(speed=(adsCtrlRate*0.3), curve_left=0.0, curve_right=0.3)
         self.setVacuumMotorPWM(0.1)
      elif self.leftDistSens.distance < 0.20:
         self.robot.stop()
         self.setVacuumMotorPWM(0.0)
         self.resetEncoders()
      elif self.leftDistSens.distance < 0.30:
         self.robot.forward(speed=(adsCtrlRate*0.3), curve_left=0.0, curve_right=0.3)
         self.setVacuumMotorPWM(0.5)
      else:
         self.robot.forward(speed=(adsCtrlRate*0.8), curve_left=0.0, curve_right=0.8)
         self.setVacuumMotorPWM(0.8)

      ### periodic message update ###
      self.updatePeriodicMsg()

      if self.tubeSensMngr.isBallStuck():
         self.periodicMsg["robot_error"] = "ball_stuck"
         self.mainMsgQueue.put(json.dumps(self.periodicMsg))
      else:
         self.periodicMsg["robot_error"] = ""

      if self.adsChannel.voltage < 1.5:
         self.periodicMsg["robot_error"] = "low_battery"
         self.mainMsgQueue.put(json.dumps(self.periodicMsg))
      else:
         self.periodicMsg["robot_error"] = ""

      self.mainMsgQueue.put(json.dumps(self.periodicMsg))

   def run(self):
      while(1):
         try:
            self.runRobot()
         except KeyboardInterrupt:
            self.safeExit(None, None)
         
         time.sleep(1)

   def start(self):
      self.thread.start()
   def stop(self):
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
      self.thread.join()

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
      return self.leftDistSens.distance
   # def getFrontDistance(self) -> float:
   #    return self.frontDistSens.distance
   # def getRightDistance(self) -> float:
   #    return self.rightDistSens.distance
   # def getBackDistance(self) -> float:
   #    return self.backDistSens.distance

   def setVacuumMotorPWM(self, pwm: float):
      if (pwm >= 0) and (pwm <= 1):
         self.vacuumMotor.blink(on_time=(0.01*pwm), off_time=(0.01*(1-pwm)))

   # Drive the robot forward by running both motors forward.
   # Left and right relative to the robot itself
   def turnRobotForward(self, speed: float, curveLeftRate: float, curveRightRate: float):
      if ((curveLeftRate < 0 or curveLeftRate > 1) or
          (curveRightRate < 0 or curveRightRate > 1) or
          (speed < 0 or speed > 1)):
         print("Invalid speed values")
         return
      
      self.robot.forward(speed=speed, curve_left=curveLeftRate, curve_right=curveRightRate)
   
   # Drive the robot backward by running both motors backward.
   # Left and right relative to the robot itself
   def turnRobotBackward(self, speed: float, curveLeftRate: float, curveRightRate: float):
      if ((curveLeftRate < 0 or curveLeftRate > 1) or
          (curveRightRate < 0 or curveRightRate > 1) or
          (speed < 0 or speed > 1)):
         print("Invalid speed values")
         return
      
      self.robot.backward(speed=speed, curve_left=curveLeftRate, curve_right=curveRightRate)
   
   def getHallEffectState(self) -> bool:
      return self.hallEffectSens.is_active
   
   def updatePeriodicMsg(self):
      self.periodicMsg["sens_dist_left"] = self.leftDistSens.distance
      # self.periodicMsg["sens_dist_front"] = self.frontDistSens.distance
      # self.periodicMsg["sens_dist_right"] = self.rightDistSens.distance
      # self.periodicMsg["sens_dist_back"] = self.backDistSens.distance
      self.periodicMsg["battery_level"] = self.adsChannel.voltage
      self.periodicMsg["balls_collected"] = self.tubeSensMngr.ballCount
      self.periodicMsg["balls_coordinates"] = []
      self.periodicMsg["robot_status"] = "collecting_balls"