import threading
import time
from gpiozero import Robot, Motor, Servo, DistanceSensor, PWMOutputDevice, RotaryEncoder

class PeripheralsTask:
   def __init__(self):
      self.name = "PeripheralsTask"
      self.description = "PeripheralsTask"
      self.dependencies = []
      # self.vacuumMotor   = PWMOutputDevice(pin=12)
      self.robot         = Robot(left=(Motor(19, 26)), right=(Motor(20, 21)))
      self.encoderLeft   = RotaryEncoder(a=5, b=6)
      # self.encoderRight  = RotaryEncoder(a=25, b=16)
      self.leftDistSens  = DistanceSensor(echo=27, trigger=17, threshold_distance=0.15)
      # self.frontDistSens = DistanceSensor(echo=22, trigger=17, threshold_distance=0.15)
      # self.rightDistSens = DistanceSensor(echo=10, trigger=17, threshold_distance=0.15)
      # self.backDistSens  = DistanceSensor(echo=9, trigger=17, threshold_distance=0.15)
      # self.servo         = Servo(pin=24)
      self.thread = threading.Thread(target=self.run)

   def runRobot(self):
      print(str(self.leftDistSens.distance) + ", steps: " + str(self.encoderLeft.steps))
      if self.leftDistSens.distance < 0.15:
         self.robot.stop()
      elif self.leftDistSens.distance < 0.30:
         self.robot.forward(speed=0.3, curve_left=0.0, curve_right=0.3)
      else:
         self.robot.forward(speed=0.8, curve_left=0.0, curve_right=0.8)

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
      self.thread.join()

   def safeExit(self, signum, frame):
      self.robot.stop()
      exit(1)

   def getLeftEncoderSteps(self) -> int:
      return self.encoderLeft.steps
   # def getRightEncoderSteps(self) -> int:
   #    return self.encoderRight.steps
   
   def getLeftDistance(self) -> float:
      return self.leftDistSens.distance
   # def getFrontDistance(self) -> float:
   #    return self.frontDistSens.distance
   # def getRightDistance(self) -> float:
   #    return self.rightDistSens.distance
   # def getBackDistance(self) -> float:
   #    return self.backDistSens.distance

   def turnRobotForward(self, speed: float, leftSpeed: float, rightSpeed: float):
      if ((leftSpeed < 0 or leftSpeed > 1) or (rightSpeed < 0 or rightSpeed > 1) or (speed < 0 or speed > 1)):
         print("Invalid speed values")
         return
      
      self.robot.forward(speed=speed, curve_left=leftSpeed, curve_right=rightSpeed)
   def turnRobotBackward(self, speed: float, leftSpeed: float, rightSpeed: float):
      if ((leftSpeed < 0 or leftSpeed > 1) or (rightSpeed < 0 or rightSpeed > 1) or (speed < 0 or speed > 1)):
         print("Invalid speed values")
         return
      
      self.robot.backward(speed=speed, curve_left=leftSpeed, curve_right=rightSpeed)
   