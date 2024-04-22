from signal import signal, SIGTERM, SIGHUP, pause
from gpiozero import Robot, Motor, Servo, DistanceSensor

robot = Robot(left=(Motor(19, 26)), right=(Motor(20, 21)))
ultrassonicSens = DistanceSensor(echo=27, trigger=17, threshold_distance=0.15)

def safeExit(signum, frame):
   robot.stop()
   exit(1)

def runRobot():
   while True:
      if ultrassonicSens.distance < 0.15:
         robot.stop()
      elif ultrassonicSens.distance < 0.30:
         robot.right()
      else:
         robot.forward()

try:
   signal(SIGTERM, safeExit)
   signal(SIGHUP, safeExit)

   robot.source = runRobot

   pause()
except KeyboardInterrupt:
   safeExit(None, None)