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
      print("running")
      # print(f"Sensor distance: {str(9.0)}, \nsteps: {str(self.peripherals.getLeftEncoderSteps())}, \ncollected balls: {str(self.peripherals.getCollectedBallsQty())}")
      # print(f"Ads value: {str(self.peripherals.getLowerBatteryADCVal())}, voltage: {str(self.peripherals.getLowerBatteryLvl())}\n")

      # if self.peripherals.isHallEffectSensActive():
      #    self.peripherals.stopRobot()
      #    self.peripherals.setVacuumMotorPWM(0.0)
      #    self.peripherals.resetEncoders()
      #    return

      # print("Batt ", self.peripherals.getLowerBatteryLvl())
      # self.peripherals.rotate(1, 1)
      print("ligando aspirador")
      self.peripherals.setVacuumMotorPWM(0.1)
      time.sleep(0.5)
      self.peripherals.setVacuumMotorPWM(0.3)
      print("Subindo rampa velocidade")
      self.peripherals.driveRobotForward(0.1, 0, 0)
      self.peripherals.controlMotorsPWM()
      time.sleep(1)
      self.peripherals.driveRobotForward(0.2, 0, 0)
      self.peripherals.controlMotorsPWM()
      time.sleep(1)
      self.peripherals.driveRobotForward(0.4, 0, 0)
      self.peripherals.controlMotorsPWM()
      time.sleep(1)
      self.peripherals.driveRobotForward(0.5, 0, 0)
      self.peripherals.controlMotorsPWM()
      time.sleep(1)
      print("parado")
      time.sleep(3)
      self.peripherals.driveRobotForward(0.0, 0, 0)
      self.peripherals.controlMotorsPWM()
      print("desliango aspirador")
      self.peripherals.setVacuumMotorPWM(0.1)
      time.sleep(0.5)
      self.peripherals.setVacuumMotorPWM(0.0)
      time.sleep(0.5)
      print("movendo para tras")
      self.peripherals.driveRobotBackward(0.1, 0, 0)
      self.peripherals.controlMotorsPWM()
      time.sleep(0.5)
      self.peripherals.driveRobotBackward(0.2, 0, 0)
      self.peripherals.controlMotorsPWM()
      time.sleep(0.5)
      self.peripherals.driveRobotBackward(0.4, 0, 0)
      self.peripherals.controlMotorsPWM()
      time.sleep(0.5)
      self.peripherals.driveRobotBackward(0.5, 0, 0)
      self.peripherals.controlMotorsPWM()
      time.sleep(0.5)
      time.sleep(3)
      self.peripherals.driveRobotBackward(0.0, 0, 0)
      self.peripherals.controlMotorsPWM()
      self.peripherals.stopRobot()
      # print("Batt ", self.peripherals.getLowerBatteryLvl())
      # self.peripherals.rotate(0, 1)
      print("FINALIZADO")

      # adsCtrlRate = 30.0 / 100.0
      # if adsCtrlRate < 0.0:
      #    adsCtrlRate = 0.0
      # elif adsCtrlRate > 1.0:
      #    adsCtrlRate = 1.0

      # if 9.0 < 0.10:
      #    self.peripherals.driveRobotBackward(speed=(adsCtrlRate*0.3), curveLeftRate=0.0, curveRightRate=0.3)
      #    self.peripherals.setVacuumMotorPWM(0.1)
      # elif 9.0 < 0.20:
      #    self.peripherals.stopRobot()
      #    self.peripherals.setVacuumMotorPWM(0.0)
      #    self.peripherals.resetEncoders()
      # elif 9.0 < 0.30:
      #    self.peripherals.driveRobotForward(speed=(adsCtrlRate*0.3), curveLeftRate=0.0, curveRightRate=0.3)
      #    self.peripherals.setVacuumMotorPWM(0.5)
      # else:
      #    self.peripherals.driveRobotForward(speed=(adsCtrlRate*0.8), curveLeftRate=0.0, curveRightRate=0.8)
      #    self.peripherals.setVacuumMotorPWM(0.8)

      self.peripherals.controlMotorsPWM()

      ### bluetooth periodic message update ###
      # print("Before updateBtPeriodicMsg")
      self.updateBtPeriodicMsg()

      # if self.peripherals.isBallStuck():
      #    self.btPeriodicMsg["robot_error"] = "ball_stuck"
      # elif 30.0 < 25.0:
      #    self.btPeriodicMsg["robot_error"] = "low_battery"
      # else:
      #    self.btPeriodicMsg["robot_error"] = ""

      print("Before put")
      self.mainMsgQueue.put(json.dumps(self.btPeriodicMsg))

   def run(self):
      # 
      while(1):
         try:
            self.runRobot()
            print("running robot")
            
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
      self.btPeriodicMsg["sens_dist_front"] = self.peripherals.getFrontDistance()
      self.btPeriodicMsg["sens_dist_right"] = self.peripherals.getRightDistance()
      self.btPeriodicMsg["sens_dist_back"] = self.peripherals.getBackDistance()
      self.btPeriodicMsg["battery_level"] = 30.0
      self.btPeriodicMsg["balls_collected"] = self.peripherals.getCollectedBallsQty()
      self.btPeriodicMsg["balls_coordinates"] = []
      self.btPeriodicMsg["robot_status"] = "collecting_balls"