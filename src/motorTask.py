import threading
import time
from signal import signal, SIGTERM, SIGHUP, pause
from gpiozero import Robot, Motor, Servo, DistanceSensor

class MotorTask:
   def __init__(self):
      self.name = "MotorTask"
      self.description = "MotorTask"
      self.dependencies = []
      # self.robot = Robot(left=(Motor(19, 26)), right=(Motor(20, 21)))
      # self.ultrassonicSens = DistanceSensor(echo=27, trigger=17, threshold_distance=0.15)
      self.thread = threading.Thread(target=self.run)

   # def runRobot(self):
   #    while True:
   #       if self.ultrassonicSens.distance < 0.15:
   #          self.robot.stop()
   #       elif self.ultrassonicSens.distance < 0.30:
   #          self.robot.right()
   #       else:
   #          self.robot.forward()

   def run(self):
      while(1):
         print("Running motor task")
         time.sleep(1)
      # try:
      #    signal(SIGTERM, self.safeExit)
      #    signal(SIGHUP, self.safeExit)

      #    self.robot.source = self.runRobot

      #    pause()
      # except KeyboardInterrupt:
      #    self.safeExit(None, None)

   def start(self):
      self.thread.start()
   def stop(self):
      self.thread.join()

   def safeExit(self, signum, frame):
      # self.robot.stop()
      exit(1)