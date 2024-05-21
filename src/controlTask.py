import threading
import time
import queue
import json
from auxClasses.peripherals import Peripherals

class ControlTask:
   def __init__(self, mainQueue: queue.Queue, ctrlQueue: queue.Queue):
      self.name           = "ControlTask"
      self.description    = "ControlTask"
      self.mainMsgQueue   = mainQueue
      self.ctrlMsgQueue   = ctrlQueue
      self.peripherals    = Peripherals()
      self.btPeriodicMsg  = {
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
      print(f"Sensor distance: {str(self.peripherals.getLeftDistance())}, \nsteps: {str(self.peripherals.getLeftEncoderSteps())}, \ncollected balls: {str(self.peripherals.getCollectedBallsQty())}")
      print(f"Ads value: {str(self.peripherals.getBatteryADCValue())}, voltage: {str(self.peripherals.getBatteryVoltage())}\n")

      if self.peripherals.isHallEffectSensActive():
         self.peripherals.stopRobot()
         self.peripherals.setVacuumMotorPWM(0.0)
         self.peripherals.resetEncoders()
         return

      adsCtrlRate = self.peripherals.getBatteryVoltage() / 3.3
      if adsCtrlRate < 0.0:
         adsCtrlRate = 0.0
      elif adsCtrlRate > 1.0:
         adsCtrlRate = 1.0

      if self.peripherals.getLeftDistance() < 0.10:
         self.peripherals.driveRobotBackward(speed=(adsCtrlRate*0.3), curveLeftRate=0.0, curveRightRate=0.3)
         self.peripherals.setVacuumMotorPWM(0.1)
      elif self.peripherals.getLeftDistance() < 0.20:
         self.peripherals.stopRobot()
         self.peripherals.setVacuumMotorPWM(0.0)
         self.peripherals.resetEncoders()
      elif self.peripherals.getLeftDistance() < 0.30:
         self.peripherals.driveRobotForward(speed=(adsCtrlRate*0.3), curveLeftRate=0.0, curveRightRate=0.3)
         self.peripherals.setVacuumMotorPWM(0.5)
      else:
         self.peripherals.driveRobotForward(speed=(adsCtrlRate*0.8), curveLeftRate=0.0, curveRightRate=0.8)
         self.peripherals.setVacuumMotorPWM(0.8)

      ### bluetooth periodic message update ###
      self.updateBtPeriodicMsg()

      if self.peripherals.isBallStuck():
         self.btPeriodicMsg["robot_error"] = "ball_stuck"
      elif self.peripherals.getBatteryVoltage() < 1.5:
         self.btPeriodicMsg["robot_error"] = "low_battery"
      else:
         self.btPeriodicMsg["robot_error"] = ""

      self.mainMsgQueue.put(json.dumps(self.btPeriodicMsg))

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
      self.peripherals.close()
      self.thread.join()

   def safeExit(self, signum, frame):
      self.peripherals.close()
      self.thread.join()
      exit(1)
   
   def updateBtPeriodicMsg(self):
      self.btPeriodicMsg["sens_dist_left"] = self.peripherals.getLeftDistance()
      # self.btPeriodicMsg["sens_dist_front"] = self.frontDistSens.distance
      # self.btPeriodicMsg["sens_dist_right"] = self.rightDistSens.distance
      # self.btPeriodicMsg["sens_dist_back"] = self.backDistSens.distance
      self.btPeriodicMsg["battery_level"] = self.peripherals.getBatteryVoltage()
      self.btPeriodicMsg["balls_collected"] = self.peripherals.getCollectedBallsQty()
      self.btPeriodicMsg["balls_coordinates"] = []
      self.btPeriodicMsg["robot_status"] = "collecting_balls"