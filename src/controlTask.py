import threading
import time
import queue
import json
from auxClasses.peripherals import Peripherals
from enum import Enum
import math
import cv2
from picamera2 import Picamera2, Preview
import numpy as np

dist = lambda x1,y1,x2,y2: math.sqrt((x1 - x2)**2 + (y1 - y2)**2)



def closest(element):
   return dist(0, 0, element[0], element[1])
   
class State(Enum):
   INIT = 0,
   CONNECTED_TO_BASE = 1,
   SEARCHING_BASE_WALL = 2,
   CONNECTING_TO_BASE = 3,
   ROBOT_STUCK = 4
   FINDING_WALL = 5,
   SEARCHING_BASE_CAM = 6,
   WAITING_FOR_CHARGER = 7,
   UNLOADING = 8,
   WAITING_FOR_CMD_OR_SCHEDULE = 9,
   CHARGING = 10,
   SEARCHING_BALLS = 11,
   AVOIDING_WALL = 12,
   AVOIDING_STATIC_OBJECT = 13,
   PAUSED = 14,
   CATCHING_BALL = 15,
   BALL_STUCK = 16,
   WAITING_USER = 17

global_map = []

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
         "robot_error": "" # Options: base_not_found, robot_stuck, ball_stuck
      }
      self.is_rotating = False
      self.balls = []
      self.cam = Picamera2(1)
      cfg = self.cam.create_preview_configuration(main={'size': (1920, 1080)})
      self.cam.configure(cfg)
      # cam.set_controls({"FrameRate": 5})
      self.cam.resolution = (1920, 1080)
      self.cam.framerate = 10
      self.last_position = [0, 0, 0]

      self.cam.start()
      self.stereo = cv2.StereoBM.create()
      self.thread = threading.Thread(target=self.run)
      self.ball_detector_th = threading.Thread(target=self.ballDetectorTh)
      self.static_obj_detector_th = threading.Thread(target=self.run)
      self.odometry_thread = threading.Thread(target=self.runOdometry)
      self.bluetooth_thread = threading.Thread(target=self.bluetoothSenderTh)

   def runOdometry(self):
      while(True):
         if(self.is_rotating == False):
            self.peripherals.controlMotorsPWM()
         self.peripherals.updatePositionOdometry()
         odom = self.peripherals.getOdometry()
         self.updateMap(odom[1]*100 - self.last_position[1], odom[0]*100 - self.last_position[0], odom[2] - self.last_position[2])
         self.last_position[0] = odom[0]*100
         self.last_position[1] = odom[1]*100
         self.last_position[2] = odom[2]
         time.sleep(0.1)
   
   def bluetoothSenderTh(self):
      while True:
         self.updateBtPeriodicMsg()
         self.mainMsgQueue.put(json.dumps(self.btPeriodicMsg))
         time.sleep(0.5)


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
      # print("ligando aspirador")
      # self.peripherals.setVacuumMotorPWM(0.1)
      # time.sleep(0.5)
      # self.peripherals.setVacuumMotorPWM(0.3)
      # print("Subindo rampa velocidade")
      # self.peripherals.driveRobotForward(0.1, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # time.sleep(1)
      # self.peripherals.driveRobotForward(0.2, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # time.sleep(1)
      # self.peripherals.driveRobotForward(0.4, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # time.sleep(1)
      # self.peripherals.driveRobotForward(0.5, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # time.sleep(1)
      # print("parado")
      # time.sleep(3)
      # self.peripherals.driveRobotForward(0.0, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # print("desliango aspirador")
      # self.peripherals.setVacuumMotorPWM(0.1)
      # time.sleep(0.5)
      # self.peripherals.setVacuumMotorPWM(0.0)
      # time.sleep(0.5)
      # print("movendo para tras")
      # self.peripherals.driveRobotBackward(0.1, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # time.sleep(1)
      # self.peripherals.driveRobotBackward(0.2, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # time.sleep(1)
      # self.peripherals.driveRobotBackward(0.4, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # time.sleep(1)
      # self.peripherals.driveRobotBackward(0.5, 0, 0)
      # # self.peripherals.controlMotorsPWM()
      # time.sleep(1)
      # time.sleep(3)
      # self.peripherals.driveRobotBackward(0.0, 0, 0)
      # self.peripherals.controlMotorsPWM()
      # self.peripherals.rotate(0, 3.1415, 0.5)
      # self.peripherals.stopRobot()
      # print("Batt ", self.peripherals.getLowerBatteryLvl())
      # self.peripherals.rotate(0, 1)

      self.approachBall()
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

      # self.peripherals.controlMotorsPWM()
      # self.peripherals.updatePositionOdometry()

      ### bluetooth periodic message update ###
      # print("Before updateBtPeriodicMsg")
      # self.updateBtPeriodicMsg()

      # if self.peripherals.isBallStuck():
      #    self.btPeriodicMsg["robot_error"] = "ball_stuck"
      # elif 30.0 < 25.0:
      #    self.btPeriodicMsg["robot_error"] = "low_battery"
      # else:
      #    self.btPeriodicMsg["robot_error"] = ""

      print("Before put")
      
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
      self.odometry_thread.start()
      self.bluetooth_thread.start()
      self.ball_detector_th.start()
   
   def stop(self):
      self.peripherals.close()
      self.thread.join()
      self.odometry_thread.join()
      self.ball_detector_th.join()
      self.bluetooth_thread.join()

   def safeExit(self, signum, frame):
      self.peripherals.close()
      self.odometry_thread.join()
      self.ball_detector_th.join()
      self.bluetooth_thread.join()
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

   def ballDetectorTh(self):
      # mean = 0
      i = 0
      while True:
         a = self.cam.capture_array("main")
         a = cv2.cvtColor(a, cv2.COLOR_BGR2RGB)
         a = cv2.resize(a, (960, 540))
         # a = cv2.rotate(a, cv2.ROTATE_180)
         
         # self.updateMap(0, 0, 0)

         if a is None:
            continue

         a_grey = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
         # a_grey_up = a_grey[300:, :]
         a_grey = a_grey[300:, :]
         a_grey = cv2.normalize(a_grey, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
         # (a_t, threshInv) = cv2.threshold(a_grey_up, 200, 255,cv2.THRESH_BINARY)
         a_blur = cv2.GaussianBlur(a_grey, (17, 17), 0)
         circles = cv2.HoughCircles(a_blur, cv2.HOUGH_GRADIENT, 1.2, 10, param1=100, param2=35, minRadius=13, maxRadius=46)
         # print(circles)
         if circles is not None:
            circles = np.uint16(np.around(circles))
            # ball_added = False
            for i in circles[0, :]:
                  print(a_grey[i[1]][i[0]])
                  if(a_grey[i[1]][i[0]] > 25):
                     cv2.circle(a_grey, (i[0], i[1]), 1, (0,100,100), 3)
                     cv2.circle(a_grey, (i[0], i[1]), i[2], (255,0,255), 3)

                     dist_ball = pow(i[2], -1.05)
                     dist_ball *= 2495
                     horizontal_dist = 2*(i[0] - 480)/i[2]
                     theta = math.asin(horizontal_dist/dist_ball)
                     dist_center_ball = math.sqrt((dist_ball * dist_ball) + (3 * 3) - 2 * dist_ball * 3 * theta)
                     theta_center = math.asin((horizontal_dist + 3)/dist_center_ball)
                     print("dist: ", i[2], " pixel")
                     print("dist: ", dist_ball, " cm")
                     print("hor dist: ", horizontal_dist, " cm")
                     print("angle with camera: ", theta*180/math.pi)
                     print("center dist: ", dist_center_ball, " cm")
                     print("angle with center: ", theta_center*180/math.pi)
                     x_ball, y_ball = self.convert(dist_center_ball, theta_center)
                     self.addBall(x_ball, y_ball)

         # verifier()
         print(self.balls)
         # cv2.imshow("iR", threshInv) 
         # cv2.waitKey(1)
         # cv2.imshow("b", b_grey)
         # cv2.waitKey(1)
         # cv2.imshow("a", a_grey)
         # cv2.imshow("an", a_norm)
         # cv2.waitKey(1)
         # print(len(a))
         
         if cv2.waitKey(1) == 27:
            break
      time.sleep(10)

   def staticObjDetectorTh(self):
      return

   def fsmInit(self):
      self.charge_complete = False
      self.opperation_complete = False
      self.close_from_base = False
      self.charger_connected = False
      self.battery_low = False
      self.in_operation = False
      self.start_command_rcvd = False
      self.start_schedule = False
      self.has_balls = False
      self.ball_detected = False
      self.static_object_detected = False
      self.close_wall = False
      self.stop_command_rcvd = False
      self.load_full = False
      self.end_schedule = False
      self.ball_caught = False
      self.static_object_avoided = False
      self.is_parallel_wall = False
      self.base_found_camera = False
      self.base_not_found = False
      self.message_sent = False
      self.base_connected = False
      self.front_ir_detected_but_not_end = False
      self.robot_running_encoder = False
      self.robot_running_imu = False
      self.resume_command_rcvd = False
      self.pause_command_rcvd = False

      self.actual_state = State.INIT
      self.next_state = State.INIT
      self.last_state = State.INIT

   def fsmRun(self):
      # Update booleans

      # Run FSM
      if(self.next_state == State.INIT):
         self.init()
      elif(self.next_state == State.CONNECTED_TO_BASE):
         self.connectedToBase()
      elif(self.next_state == State.SEARCHING_BASE_WALL):
         self.searchingBaseFollowingWall()
      elif(self.next_state == State.CONNECTING_TO_BASE):
         self.connectingToBase()
      elif(self.next_state == State.FINDING_WALL):
         self.findingWall()
      elif(self.next_state == State.SEARCHING_BASE_CAM):
         self.searchingBaseUsingCamera()
      elif(self.next_state == State.ROBOT_STUCK):
         self.robotStuck()
      elif(self.next_state == State.WAITING_FOR_CHARGER):
         self.waitingForCharger()
      elif(self.next_state == State.UNLOADING):
         self.unloading()
      elif(self.next_state == State.WAITING_FOR_CMD_OR_SCHEDULE):
         self.waitingForStartCommandOrSchedule()
      elif(self.next_state == State.CHARGING):
         self.charging()
      elif(self.next_state == State.SEARCHING_BALLS):
         self.searchingForBall()
      elif(self.next_state == State.AVOIDING_WALL):
         self.avoidingWall()
      elif(self.next_state == State.AVOIDING_STATIC_OBJECT):
         self.avoidingStaticObject()
      elif(self.next_state == State.PAUSED):
         self.robotPaused()
      elif(self.next_state == State.CATCHING_BALL):
         self.catchingBall()
      elif(self.next_state == State.BALL_STUCK):
         self.ballStuck()
      elif(self.next_state == State.WAITING_USER):
         self.waitForUser()

   def init(self):
      # Entry
      if(self.actual_state == State.INIT):
         self.actual_state = self.next_state
      
      # Exit
      if(self.charger_connected == True):
         self.next_state = State.CHARGING
         self.last_state = self.actual_state
      elif(self.base_connected == True):
         self.next_state = State.CONNECTED_TO_BASE
         self.last_state = self.actual_state
      elif(self.base_connected == False):
         self.next_state = State.SEARCHING_BASE_WALL
         self.last_state = self.actual_state
      
   def searchingBaseFollowingWall(self):
      # Entry
      if(self.actual_state != State.SEARCHING_BASE_WALL):
         self.actual_state = self.next_state
      
      # Do
      self.moveParallelWall()
      self.findIR()
      
      # Exit
      if(self.close_from_base == True):
         self.next_state = State.CONNECTING_TO_BASE
         self.last_state = self.actual_state
      elif(self.close_wall == False):
         self.next_state = State.FINDING_WALL
         self.last_state = self.actual_state
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
   
   def findingWall(self):
      # Entry
      if(self.actual_state != State.FINDING_WALL):
         self.actual_state = self.next_state
      
      # Do
      self.findWall()
      
      # Exit
      if(self.close_wall == True):
         self.next_state = State.SEARCHING_BASE_WALL
         self.last_state = self.actual_state
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state

   def connectingToBase(self):
      # Entry
      if(self.actual_state != State.CONNECTING_TO_BASE):
         self.actual_state = self.next_state
      
      # Do
      self.steerBase()

      # Exit
      if(self.base_connected == True):
         self.next_state = State.CONNECTED_TO_BASE
         self.last_state = self.actual_state
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state

   def searchingBaseUsingCamera(self):
      ## REVIEW THIS BEFORE CONTINUE
      # Entry
      if(self.actual_state != State.SEARCHING_BASE_CAM):
         self.actual_state = self.next_state
      
      # Do
      self.findIR()

      # Exit
      if(self.base_not_found == True):
         self.next_state = State.SEARCHING_BASE_WALL
         self.last_state = self.actual_state
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state

   def connectedToBase(self):
      # Entry
      if(self.actual_state != State.CONNECTED_TO_BASE):
         self.actual_state = self.next_state
      
      # Do
      self.calibrateSensors()
      
      # Exit
      if(self.battery_low == True):
         self.next_state = State.WAITING_FOR_CHARGER
         self.last_state = self.actual_state
      elif(self.has_balls):
         self.next_state = State.UNLOADING
         self.last_state = self.actual_state
      elif(self.opperation_complete == True):
         self.next_state = State.WAITING_FOR_CMD_OR_SCHEDULE
         self.last_state = self.actual_state
   
   def waitingForCharger(self):
      # Entry
      if(self.actual_state != State.WAITING_FOR_CHARGER):
         self.actual_state = self.next_state
      
      # Do
      
      # Exit
      if(self.charger_connected == True):
         self.next_state = State.CHARGING
         self.last_state = self.actual_state

   def unloading(self):
      # Entry
      if(self.actual_state != State.UNLOADING):
         start_time = time.time()
         self.actual_state = self.next_state
      
      # Do
      
      # Exit
      if(time.time() - start_time >= 15):
         self.next_state = State.CONNECTED_TO_BASE
         self.last_state = self.actual_state
   
   def charging(self):
      # Entry
      if(self.actual_state != State.WAITING_FOR_CHARGER):
         self.disablePeripherals()
         self.actual_state = self.next_state
      
      # Do

      
      # Exit
      if(self.charge_complete == True and self.charger_connected == False):
         self.enablePeripherals()
         self.next_state = State.CONNECTED_TO_BASE
         self.last_state = self.actual_state
   
   def waitingForStartCommandOrSchedule(self):
      # Entry
      if(self.actual_state != State.WAITING_FOR_CMD_OR_SCHEDULE):
         self.disablePeripherals()
         self.actual_state = self.next_state
      
      # Do

      
      # Exit
      if(self.start_command_rcvd == True or self.start_schedule == True):
         self.enablePeripherals()
         self.next_state = State.SEARCHING_BALLS
         self.last_state = self.actual_state
   
   def searchingForBall(self):
      # Entry
      if(self.actual_state != State.SEARCHING_BALLS):
         self.actual_state = self.next_state
      
      # Do
      self.moveInPattern()
      
      # Exit
      if(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
      elif(self.close_wall == True):
         self.next_state = State.AVOIDING_WALL
         self.last_state = self.actual_state
      elif(self.static_object_detected == True):
         self.next_state = State.AVOIDING_STATIC_OBJECT
         self.last_state = self.actual_state
      elif(self.pause_command_rcvd == True):
         self.next_state = State.PAUSED
         self.last_state = self.actual_state
      elif(self.stop_command_rcvd == True or self.battery_low == True or self.load_full == True or self.end_schedule == True):
         self.next_state = State.SEARCHING_BASE_CAM
         self.last_state = self.actual_state
      elif(self.ball_detected == True):
         self.next_state = State.CATCHING_BALL
         self.last_state = self.actual_state
      
   def avoidingWall(self):
      # Entry
      if(self.actual_state != State.AVOIDING_WALL):
         self.actual_state = self.next_state
      
      # Do
      self.moveParallelWall()
      
      # Exit
      if(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
      elif(self.close_wall == True):
         self.next_state = State.SEARCHING_BALLS
         self.last_state = self.actual_state
      elif(self.pause_command_rcvd == True):
         self.next_state = State.PAUSED
         self.last_state = self.actual_state

   def avoidingStaticObject(self):
      if(self.actual_state != State.AVOIDING_STATIC_OBJECT):
         self.reduceSpeed()
         self.actual_state = self.next_state
      
      # Do
      self.moveAroundObject()
      
      # Exit
      if(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
      elif(self.static_object_avoided == True):
         self.increaseSpeed()
         self.next_state = State.SEARCHING_BALLS
         self.last_state = self.actual_state
      elif(self.pause_command_rcvd == True):
         self.next_state = State.PAUSED
         self.last_state = self.actual_state
   
   def robotPaused(self):
      if(self.actual_state != State.AVOIDING_STATIC_OBJECT):
         self.stopMotors()
         self.actual_state = self.next_state
      
      # Do
      
      # Exit
      if(self.static_object_avoided == True):
         self.next_state = self.last_state
         self.last_state = self.actual_state
   
   def catchingBall(self):
      if(self.actual_state != State.CATCHING_BALL):
         self.reduceSpeed()
         self.increaseVacuumPower()
         self.actual_state = self.next_state
      
      # Do
      self.approachBall()
      
      # Exit
      if(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
      elif(self.ball_caught == True):
         self.increaseSpeed()
         self.reduceVacuumPower()
         self.next_state = State.SEARCHING_BALLS
         self.last_state = self.actual_state
      elif(self.front_ir_detected_but_not_end == True):
         self.next_state = State.BALL_STUCK
         self.last_state = self.actual_state
      elif(self.pause_command_rcvd == True):
         self.next_state = State.PAUSED
         self.last_state = self.actual_state

   def ballStuck(self):
      if(self.actual_state != State.BALL_STUCK):
         self.stopMotors()
         start_time = time.time()
         self.actual_state = self.next_state
      
      # Do
      self.sendWarningUser()

      # Exit
      if(self.front_ir_detected_but_not_end == True):
         self.next_state = State.SEARCHING_BALLS
         self.last_state = self.actual_state
      elif(time.time() - start_time > 3):
         self.next_state = State.WAITING_USER
         self.last_state = self.actual_state

   def waitForUser(self):
      if(self.actual_state != State.WAITING_USER):
         self.stopMotors()
         self.actual_state = self.next_state
      
      # Do
      
      # Exit
   
   def robotStuck(self):
      print("Not implemented")
      return

   def baseNotFound(self):
      print("Not implemented")
      return

   def findWall(self):
      print("Not implemented")
      return

   def moveParallelWall(self):
      print("Not implemented")
      return

   def steerBase(self):
      print("Not implemented")
      return
   
   def findIR(self):
      print("Not implemented")
      return
   
   def calibrateSensors(self):
      print("Not implemented")
      return
   
   def enablePeripherals(self):
      print("Not implemented")
      return

   def disablePeripherals(self):
      print("Not implemented")
      return
   
   def moveInPattern(self):
      print("Not implemented")
      return

   def reduceSpeed(self):
      print("Not implemented")
      return

   def moveAroundObject(self):
      print("Not implemented")
      return
   
   def increaseSpeed(self):
      print("Not implemented")
      return
   
   def stopMotors(self):
      print("Not implemented")
      return
   
   def increaseVacuumPower(self):
      print("Not implemented")
      return

   def reduceVacuumPower(self):
      print("Not implemented")
      return
   
   def approachBall(self):
      if(len(self.balls) == 0):
         return
      ball = self.balls[0]

      # self.is_rotating = True
      # time.sleep(1)
      # while(ball[0] > 2 or ball[0] < -2):
      #    angle = math.atan(ball[0]/ball[1])
      #    if(ball[0] < 0):
      #       self.peripherals.rotate(0, 0.1, 1)
      #    else:
      #       self.peripherals.rotate(1, 0.1, 1)
      #    time.sleep(0.01)
      # self.is_rotating = False
      self.peripherals.driveRobotForward(0.1, 0, 0)
      time.sleep(0.3)
      self.peripherals.driveRobotForward(0.2, 0, 0)
      time.sleep(0.3)
      self.peripherals.driveRobotForward(0.3, 0, 0)
      time.sleep(0.3)
      while(ball[1] > 24):
         self.peripherals.driveRobotForward(0.5, 0, 0)
         time.sleep(0.01)
      
      self.peripherals.driveRobotForward(0.1, 0, 0)
      self.peripherals.setVacuumMotorPWM(0.1)
      time.sleep(0.5)
      self.peripherals.setVacuumMotorPWM(0.3)
      time.sleep(1)

      while(ball[1] > 8):
         self.peripherals.driveRobotForward(0.3, 0, 0)
         time.sleep(0.01)
         
      self.peripherals.driveRobotForward(0.0, 0, 0)
      self.peripherals.setVacuumMotorPWM(0.0)

      return
         
   
   def sendWarningUser(self):
      print("Not implemented")
      return

   def convert(self, dist, theta):
      x = dist*math.sin(theta)
      y = dist*math.cos(theta)
      return (x, y)

   def verifier(self):
      for i in range(0, len(self.balls)):
         if(i > len(self.balls)):
               break
         if(self.balls[i][1] < 0):
               self.balls.pop(i)
               i -= 1
         elif(self.balls[i][1] > 50):
               self.balls[i][3] += 1
               if(self.balls[i][3] > 20):
                  self.balls.pop(i)
                  i -= 1

   def addBall(self, x, y):
      for ball in self.balls:
         dist_x = math.fabs(x - ball[0])
         dist_y = math.fabs(y - ball[1])
         if(dist_x < ball[2] and dist_y < ball[2]):
               ball[0] = x
               ball[1] = y
               dist_calc = dist(x,y, 0, 0)
               print(dist_calc)
               ball[2] = dist_calc*10.5/50 - 7
               ball[3] = 0
               return
      dist_calc = dist(x,y, 0, 0)
      self.balls.append([x, y, dist_calc*10.5/50 - 7, 0])
      global_map.append([x + self.last_position[0], y + self.last_position[1], dist_calc*10.5/50 - 7, 0])
      # self.global_map.sort(key=closest)
      
   def updateMap(self, delta_x, delta_y, delta_theta):
      mat = [[math.cos(delta_theta), -math.sin(delta_theta)], [math.sin(delta_theta), math.cos(delta_theta)]]
      for ball in self.balls:
         # ball[0] = ball[0]*mat[0][0] + ball[1]*mat[1][0]
         # ball[1] = ball[0]*mat[0][1] + ball[1]*mat[1][1]
         ball[0] -= delta_x
         ball[1] -= delta_y

      # self.balls.sort(key=closest())