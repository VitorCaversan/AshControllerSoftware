from gpiozero import LineSensor
import time

class InfraredSensorMngr:
   def __init__(self, frontPin, backPin):
      self.frontSensor      = LineSensor(frontPin)
      self.backSensor       = LineSensor(backPin)
      self.frontSensFiredOn = 0
      self.ballCount        = 0
      self.frontSensor.when_line = self.whenFrontFired
      self.backSensor.when_line  = self.whenBackFired

   def whenFrontFired(self):
      self.frontSensFiredOn = time.time_ns() // 1000000 # convert to milliseconds
      print(f"Ball entered")

   def whenBackFired(self):
      print(f"Ball exited")
      if self.frontSensFiredOn != 0:
         print(f"Ball {self.ballCount} took {time.time_ns() // 1000000 - self.frontSensFiredOn} ms to pass")
         self.frontSensFiredOn = 0
         self.ballCount       += 1

   def close(self):
      self.frontSensor.close()
      self.backSensor.close()