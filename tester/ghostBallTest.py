import math

balls = [[[10, 10], [20, 30, 5]], [[10, 9], [20, 40, 5]]]
mock_balls = [[20, 30, 5], [10, 40, 5], [20, 30, 7]]

dist = lambda x1,y1,x2,y2: math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def isBallsColliding(ball1: list, ball2:list):
    dist_center = dist(ball1[0], ball1[1], ball2[0], ball2[1])
    return dist_center < (ball1[2] + ball2[2])


for mock_ball in mock_balls:
    if(isBallsColliding(mock_ball, balls[1][1])):
        print("colliding")
    else:
        print("Not coliding")
        

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

IR_LED_DIST_FROM_BASE_CENTER_IN_PIXELS = 135
IR_CENTER_THRESHOLD_IN_PIXELS = 20
IR_BALL_HEIGHT_FOR_BASE_CONNECT = 110

PWM_FOR_MAX_SPEED = 0.8
PWM_FOR_MIN_SPEED = 0.2
WHEEL_DIAMETER_IN_M = 0.068
APPROX_PI = 3.141592
MAX_MOTOR_RMP = 100
WHEEL_CIRCUMFERENCE_IN_M = 3.141592 * WHEEL_DIAMETER_IN_M

LEFT_DIST_QUEUE_SIZE = 5
DIST_TOO_CLOSE_IN_CM = 30

PWM_FORWARD = 0.5
PWM_ROTATE = 0.4

IR_GHOST_BALL_DIST = 100

# Enum for forward and backward directions
class Direction(Enum):
   BACKWARD = 0
   FORWARD = 1


dist = lambda x1,y1,x2,y2: math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

global_map = []

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
   WAITING_USER = 17,
   BASE_NOT_FOUND = 18
   GOING_AFTER_BASE_CAM = 19
   ROTATING_TILL_PARALLEL = 20
   ROTATE_TEST = 21

class Position(Enum):
   X = 0,
   Y = 1,
   THETA = 2

class CyclicQueue:
   def __init__(self, size: int):
      self.size = size
      self.queue = []
      self.index = 0

   def push(self, element: float):
      if len(self.queue) < self.size:
         self.queue.append(element)
      else:
         self.queue[self.index] = element
         self.index += 1
         self.index = self.index % self.size

   def getLength(self):
      return len(self.queue)
   
   def getAvg(self):
      return sum(self.queue)/len(self.queue)

   def getQueue(self):
      return self.queue
   
   def reset(self):
      self.queue = []
      self.index = 0

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
      self.status = ""
      self.balls = []
      self.baseIRPosition = [0, 0]
      self.cam = Picamera2(0) # Right camera
      self.cam1 = Picamera2(1) # Left camera
      cfg = self.cam.create_preview_configuration(main={'size': (960, 700)})
      cfg1 = self.cam1.create_preview_configuration(main={'size': (960, 700)})
      self.cam.configure(cfg)
      self.cam1.configure(cfg1)
      # cam.set_controls({"FrameRate": 5})
      self.cam.resolution = (1920, 1080)
      self.cam1.resolution = (1920, 1080)
      self.cam.framerate = 10
      self.cam1.framerate = 10
      self.last_position = [0, 0, 0]
      self.lastRotationDir = 0
      self.lastTimeFarFromBase = 0.0
      self.starting_err_time = 0.0
      self.rotat_ctr_since_finding_wall = 0
      self.leftDistQueue = CyclicQueue(LEFT_DIST_QUEUE_SIZE)

      self.cam.start()
      self.cam1.start()
      self.stereo = cv2.StereoSGBM_create()
      self.stereo.setNumDisparities(160)
      self.stereo.setBlockSize(9)
      self.stereo.setUniquenessRatio(5)
      self.stereo.setSpeckleRange(18)
      self.stereo.setSpeckleWindowSize(7)
      self.stereo.setDisp12MaxDiff(0)
      self.stereo.setMinDisparity(110)
      self.stereo.setPreFilterCap(5)

      self.fsmInit()
      self.thread = threading.Thread(target=self.run)
      self.ball_detector_th = threading.Thread(target=self.ballDetectorTh)
      #self.static_obj_detector_th = threading.Thread(target=self.staticObjDetectorTh)
      self.odometry_thread = threading.Thread(target=self.runOdometry)
      self.bluetooth_thread = threading.Thread(target=self.bluetoothHandlerTh)

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
         time.sleep(0.05)
   
   # Handles received bluetooth messages and sends periodic messages
   def bluetoothHandlerTh(self):
      while True:
         self.updateBtPeriodicMsg()
         self.mainMsgQueue.put(json.dumps(self.btPeriodicMsg))

         try:
            msg = self.ctrlMsgQueue.get(timeout=1.0)

            if msg != "":
               print(f"new msg: {msg}")
               if msg == "pause":
                  self.pause_command_rcvd = True
               elif msg == "resume":
                  self.resume_command_rcvd = True
               elif msg == "return_to_base":
                  self.stop_command_rcvd = True
               elif msg == "start":
                  self.start_command_rcvd = True
               elif msg == "connection_lost":
                  self.pause_command_rcvd = True

               self.ctrlMsgQueue.task_done()
         except:
            pass
         
         time.sleep(0.5)

   def runRobot(self):
      print("running")
      # print(f"Sensor distance: {str(9.0)}, \nsteps: {str(self.peripherals.getLeftEncoderSteps())}, \ncollected balls: {str(self.peripherals.getCollectedBallsQty())}")
      # print(f"Ads value: {str(self.peripherals.getLowerBatteryADCVal())}, voltage: {str(self.peripherals.getLowerBatteryLvl())}\n")

      self.fsmInit()

      self.actual_state = State.SEARCHING_BALLS
      self.next_state = State.SEARCHING_BALLS
      self.last_state = State.SEARCHING_BALLS

      while(True):
         # print(f"Estado atual {self.actual_state}")
         # print(f"Próximo estado {self.next_state}")
         self.fsmRun()
         # self.rotateTest()
         time.sleep(0.01)

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
      # self.static_obj_detector_th.start()
   
   def stop(self):
      self.peripherals.close()
      self.thread.join()
      self.odometry_thread.join()
      self.ball_detector_th.join()
      self.bluetooth_thread.join()
      # self.static_obj_detector_th.join()

   def safeExit(self, signum, frame):
      self.peripherals.close()
      self.odometry_thread.join()
      self.ball_detector_th.join()
      self.bluetooth_thread.join()
      self.static_obj_detector_th.join()
      exit(1)
   
   def updateBtPeriodicMsg(self):
      self.btPeriodicMsg["sens_dist_left"] = round(self.peripherals.getLeftDistance(), 2)
      self.btPeriodicMsg["sens_dist_front"] = round(self.peripherals.getFrontDistance(), 2)
      self.btPeriodicMsg["sens_dist_right"] = round(self.peripherals.getRightDistance(), 2)
      self.btPeriodicMsg["sens_dist_back"] = round(self.peripherals.getBackDistance(), 2)
      self.btPeriodicMsg["battery_level"] = 30.0
      self.btPeriodicMsg["balls_collected"] = self.peripherals.getCollectedBallsQty()
      self.btPeriodicMsg["balls_coordinates"] = []
      for ball in self.balls:
         self.btPeriodicMsg["balls_coordinates"].append([ball[0], ball[1]])
      self.btPeriodicMsg["robot_status"] = self.status

   def ballDetectorTh(self):
      # mean = 0
      i = 0
      while True:
         a = self.cam1.capture_array("main")
         a = cv2.cvtColor(a, cv2.COLOR_BGR2RGB)
         a = cv2.rotate(a, cv2.ROTATE_180)
         # a = cv2.resize(a, (960, 540))

         infraredImg = self.cam.capture_array("main")
         infraredImg = cv2.cvtColor(infraredImg, cv2.COLOR_BGR2RGB)
         infraredImg = cv2.rotate(infraredImg, cv2.ROTATE_180)
         
         # self.updateMap(0, 0, 0)

         if (a is None) or (infraredImg is None):
            continue

         a_grey = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
         ir_gray = cv2.cvtColor(infraredImg, cv2.COLOR_RGB2GRAY)
         
         # Creates a binary image for the infrared detection. Blurrs the image and finds the circles
         # using the HoughCircles method from OpenCV
         if(self.search_for_IR == True):
            ir_gray_up = ir_gray[:400, :]
            (a_t, ir_binary_img) = cv2.threshold(ir_gray_up, 235, 255,cv2.THRESH_BINARY)
            
            ir_blur_img = cv2.GaussianBlur(ir_binary_img, (17, 17), 0)
            ir_balls = cv2.HoughCircles(ir_blur_img, cv2.HOUGH_GRADIENT, 1.3, 10, param1=50, param2=20, minRadius=4, maxRadius=20)
            #ir_balls = cv2.HoughCircles(ir_binary_img, cv2.HOUGH_GRADIENT_ALT, 1.5, 10, param1=300, param2=0.8, minRadius=3, maxRadius=25)

            if ir_balls is not None:
               ir_balls = np.uint16(np.around(ir_balls))
               for ball in ir_balls[0, :]:
                  if (ir_binary_img[ball[1]][ball[0]] > 150): 
                     if (self.baseIRPosition[1] == 0) or not (abs(self.baseIRPosition[1] - ball[1]) > IR_GHOST_BALL_DIST):
                        print(f"IR ball detected at: {ball[0] - 440}, {ball[1]}")
                        self.baseIRPosition = [(ball[0] - 440), ball[1]]
                        self.time_findIR = time.time()
                        cv2.circle(ir_blur_img, (ball[0], ball[1]), 1, (0,100,100), 3)
                        cv2.circle(ir_blur_img, (ball[0], ball[1]), ball[2], (255,0,255), 3)
            
               # cv2.imshow("iR", ir_blur_img)
               # cv2.imwrite('img_ir_gray.png', ir_gray_up)
               cv2.imwrite('img_ir.png', ir_blur_img)
               # cv2.imwrite('img_ir_bin.png', ir_binary_img)
               # cv2.waitKey(1)
            elif ((time.time() - self.time_findIR) > 4):
               self.baseIRPosition = [0, 0]
            
            if(time.time() - self.time_findIR > 45):
               self.base_not_found = True
               self.search_for_IR = False
         
         if self.stop_ball_search == False:
            a_grey = a_grey[300:, :]
            a_grey = cv2.normalize(a_grey, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            a_blur = cv2.GaussianBlur(a_grey, (17, 17), 0)
            circles = cv2.HoughCircles(a_blur, cv2.HOUGH_GRADIENT, 1.2, 10, param1=100, param2=35, minRadius=25, maxRadius=70)
            # print(circles)
            if circles is not None:
               circles = np.uint16(np.around(circles))
               # ball_added = False
               for i in circles[0, :]:
                     # print(a_grey[i[1]][i[0]])
                     if(a_grey[i[1]][i[0]] > 25):
                        cv2.circle(a_grey, (i[0], i[1]), 1, (0,100,100), 3)
                        cv2.circle(a_grey, (i[0], i[1]), i[2], (255,0,255), 3)

                        dist_ball = pow(i[2], -1.05)
                        dist_ball *= 1750
                        horizontal_dist = 2*(i[0] - 480)/i[2]
                        theta = math.asin(horizontal_dist/dist_ball)
                        dist_center_ball = math.sqrt((dist_ball * dist_ball) + (3 * 3) - 2 * dist_ball * 3 * theta)
                        theta_center = math.asin((horizontal_dist - 1)/dist_center_ball)
                        print("dist: ", i[2], " pixel")
                        print("dist: ", dist_ball, " cm")
                        print("hor dist: ", horizontal_dist, " cm")
                        print("angle with camera: ", theta*180/math.pi)
                        print("center dist: ", dist_center_ball, " cm")
                        print("angle with center: ", theta_center*180/math.pi)
                        x_ball, y_ball = self.convert(dist_center_ball, theta_center)
                        if dist_ball < 45:
                           self.addBall(x_ball, y_ball)
               
               cv2.imwrite('img.png', a_grey)

         # verifier()
         # print(self.balls)
         # cv2.imshow("iR", threshInv) 
         # cv2.waitKey(1)
         # cv2.imshow("b", b_grey)
         # cv2.waitKey(1)
         # cv2.imshow("a", a_grey)
         # cv2.imshow("an", a_norm)
         # cv2.imwrite('img.png', a_grey)
         # cv2.waitKey(1)
         # print(len(a))
         
         if cv2.waitKey(1) == 27:
            break
      time.sleep(10)

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
      self.base_connected = False
      self.front_ir_detected_but_not_end = False
      self.robot_running_encoder = False
      self.robot_running_imu = False
      self.resume_command_rcvd = False
      self.pause_command_rcvd = False
      self.balls_colected = self.peripherals.getCollectedBallsQty()
      self.encoder_left_last = self.peripherals.getLeftEncoderSteps()
      self.encoder_right_last = self.peripherals.getRightEncoderSteps()
      self.object_in_right = False
      self.object_in_left = False
      self.search_for_IR = False
      self.stop_ball_search = False
      
      self.actual_state = State.INIT
      self.next_state = State.SEARCHING_BALLS
      self.last_state = State.INIT

   def fsmRun(self):
      # Update booleans
      self.charge_complete = False           # Needs ADS
      self.opperation_complete = False       # How to define a complete operation??
      self.close_from_base = False           # Needs IR detection
      self.charger_connected = False         # Needs ADS
      self.battery_low = False               # Needs ADS     
      self.in_operation = False              # IDK
      self.start_command_rcvd = False        # IDK
      self.start_schedule = False            # IDK
      self.has_balls = self.peripherals.getCollectedBallsQty() > 0
      self.ball_detected = len(self.balls) > 0
      # self.static_object_detected = False
      self.close_wall = self.peripherals.getFrontDistance() < DIST_TOO_CLOSE_IN_CM
      # self.stop_command_rcvd = False         # IDK
      self.load_full = self.peripherals.getCollectedBallsQty() > 10
      self.end_schedule = False              # IDK
      self.ball_caught = (self.peripherals.getCollectedBallsQty() - self.balls_colected) > 0
      self.is_parallel_wall = self.peripherals.getLeftDistance() < DIST_TOO_CLOSE_IN_CM
      self.base_found_camera = False         # Needs IR detection
      # self.base_not_found = False
      self.base_connected = self.peripherals.isHallEffectSensActive()
      self.front_ir_detected_but_not_end = self.peripherals.isBallStuck()
      self.robot_running_encoder = (self.peripherals.getLeftEncoderSteps() - self.encoder_left_last) != 0 and (self.peripherals.getRightEncoderSteps() - self.encoder_right_last) != 0
      self.robot_running_imu = True         # Needs IMU
      # self.resume_command_rcvd = False       # IDK
      # self.pause_command_rcvd = False        # IDK
      self.balls_colected = self.peripherals.getCollectedBallsQty()
      self.encoder_left_last = self.peripherals.getLeftEncoderSteps()
      self.encoder_right_last = self.peripherals.getRightEncoderSteps()
      
      
      # Run FSM
      if(self.next_state == State.INIT):
         self.init()
      elif(self.next_state == State.CONNECTING_TO_BASE):
         self.connectingToBase()
      elif(self.next_state == State.FINDING_WALL):
         self.findingWall()
      elif(self.next_state == State.ROTATING_TILL_PARALLEL):
         self.rotateTillParallel()
      elif(self.next_state == State.SEARCHING_BASE_CAM):
         self.searchingBaseUsingCamera()
      elif(self.next_state == State.WAITING_FOR_CHARGER):
         self.waitingForCharger()
      elif(self.next_state == State.CHARGING):
         self.charging()
      elif(self.next_state == State.SEARCHING_BALLS):
         self.searchingForBall()

      time.sleep(0.05)

   def init(self):
      # Entry
      if(self.actual_state != State.INIT):
         self.actual_state = self.next_state
      
      # Exit
      if(self.charger_connected == True):
         self.next_state = State.CHARGING
         self.last_state = self.actual_state
      elif(self.base_connected == True):
         self.next_state = State.CONNECTED_TO_BASE
         self.last_state = self.actual_state
      elif(self.base_connected == False):
         self.next_state = State.FINDING_WALL
         self.last_state = self.actual_state
   
   # Increase one state, moving to base
   def searchingBaseUsingCamera(self):
      # Entry
      if(self.actual_state != State.SEARCHING_BASE_CAM):
         self.rotateInDirectOfBase()
         # self.is_rotating = True
         # self.peripherals.rotate(0, APPROX_PI, PWM_ROTATE) # Rotates 180 degrees
         # self.is_rotating = False
         self.actual_state = self.next_state
         self.status = "returning_to_base"
      
      # Do
      self.findIR()

      self.is_rotating = True
      print(f"base position: {self.baseIRPosition}")
      if self.baseIRPosition[0] == 0:
         self.peripherals.rotate(self.lastRotationDir, 0.03, PWM_ROTATE)
         time.sleep(0.2)
      self.is_rotating = False

      # Exit
      if self.baseIRPosition[0] != 0:
         self.next_state = State.GOING_AFTER_BASE_CAM
         self.last_state = self.actual_state
      elif(self.base_not_found == True):
         self.base_not_found = False
         self.next_state = State.FINDING_WALL
         self.last_state = self.actual_state
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
         self.status = ""

   def searchingForBall(self):
      # Entry
    #   if(self.actual_state != State.SEARCHING_BALLS):
    #      self.actual_state = self.next_state
    #      self.peripherals.driveRobotForward(0.1, 0, 0)
    #      time.sleep(0.3)
    #      self.peripherals.driveRobotForward(0.2, 0, 0)
    #      time.sleep(0.3)
    #      self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
    #      time.sleep(0.3)
    #      self.status = "searching_for_balls"
      
      # Do
      self.moveInPattern()
      
      # Exit
    #   if(self.robot_running_encoder == True and self.robot_running_imu == False):
    #      self.next_state = State.ROBOT_STUCK
    #      self.last_state = self.actual_state
    #      self.status = ""
    #   elif(self.stop_command_rcvd == True or self.battery_low == True or self.load_full == True or self.end_schedule == True):
    #      self.next_state = State.SEARCHING_BASE_CAM
    #      self.last_state = self.actual_state
    #      self.status = ""
    #      self.stop_command_rcvd = False
    #   elif(self.pause_command_rcvd == True):
    #      self.next_state = State.PAUSED
    #      self.last_state = self.actual_state
    #      self.pause_command_rcvd = False
    #      self.status = ""
    #   elif(self.static_object_detected == True):
    #      self.next_state = State.AVOIDING_STATIC_OBJECT
    #      self.last_state = self.actual_state
    #      self.status = ""
    #   elif(self.ball_detected == True):
    #      self.next_state = State.CATCHING_BALL
    #      self.last_state = self.actual_state
    #      self.stop_ball_search = True
    #      self.status = ""
      
   def avoidingWall(self):
      # Entry
      if(self.actual_state != State.AVOIDING_WALL):
         self.actual_state = self.next_state
      
      # Do
      self.leftDistQueue.push(self.peripherals.getLeftDistance())
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
         self.pause_command_rcvd = False

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
         self.pause_command_rcvd = False
   
   def findIR(self):
      if(self.search_for_IR == False):
         self.search_for_IR = True
         self.time_findIR = time.time()
   
   def calibrateSensors(self):
      self.last_position = [0, 0, 0]
      self.peripherals.resetPeripherals()
   
   def enablePeripherals(self):
      self.peripherals.start()

   def disablePeripherals(self):
      self.peripherals.close()
   
   def moveInPattern(self):
        self.peripherals.rotate(1, APPROX_PI / 2, PWM_ROTATE)
        time.sleep(1)
        self.peripherals.rotate(0, APPROX_PI / 2, PWM_ROTATE)

    #   if self.peripherals.getFrontDistance() < 30:
    #      self.is_rotating = True
    #      self.peripherals.rotate(1, APPROX_PI / 2, PWM_ROTATE)
    #      self.is_rotating = False

    #      self.peripherals.driveRobotForward(0.1, 0, 0)
    #      time.sleep(0.3)
    #      self.peripherals.driveRobotForward(0.2, 0, 0)
    #      time.sleep(0.3)
    #      self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
    #      time.sleep(0.3)

    #   if self.balls != []:
    #      self.next_state = State.CATCHING_BALL

        return

   # Improve this
   def moveAroundObject(self):
      self.static_object_avoided = False
      while(self.peripherals.getFrontDistance() < 15):
         self.peripherals.driveRobotBackward(0.2, 0, 0)
      self.is_rotating = True
      last_dist = 0
      self.peripherals.driveRobotBackward(0.0, 0, 0)
      while(self.peripherals.getFrontDistance() < 20):
         self.peripherals.rotate(0,0.1,PWM_ROTATE-0.1)
         last_dist = self.peripherals.getFrontDistance()
      d2 = last_dist*math.sin(math.pi/12)
      d3 = last_dist*math.cos(math.pi/12)
      # print(d2)
      # print(d3)
      d4 = 12 - d2
      if(d4 > 0):
         theta = math.atan(d4/d3)
         self.peripherals.rotate(0,theta*2,PWM_ROTATE-0.1)
      # time.sleep(1)
      self.is_rotating = False
      self.static_object_avoided = True
   
   def stopMotors(self):
      self.peripherals.setVacuumMotorPWM(0)
      self.peripherals.driveRobotForward(0, 0, 0)
   
   # Improve
   def approachBall(self):
      if(len(self.balls) == 0):
         return
      
      ball = self.balls[0]
      angle = math.atan(ball[0]/ball[1])
      angle = abs(angle)
      angle_sum = 0

      # ball[0] = ball[0] + 3
      
      time.sleep(1)
      while(ball[0] > 1 or ball[0] < -1):
         print(f"ball zero: {ball[0]}")
         self.is_rotating = True
         if(ball[0] > 0):
            self.peripherals.rotate(1, 0.01, PWM_ROTATE-0.05)
         else:
            self.peripherals.rotate(0, 0.01, PWM_ROTATE-0.05)
         self.is_rotating = False
         angle_sum += 0.01
         if ((angle_sum > (angle - (angle/8))) and ball[0] < -1):
            break
         if ((angle_sum > (angle - (angle/7))) and ball[0] > 1):
            break
         time.sleep(0.1)
      self.peripherals.driveRobotForward(0.1, 0, 0)
      time.sleep(0.3)
      self.peripherals.driveRobotForward(0.2, 0, 0)
      time.sleep(0.3)
      self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
      time.sleep(0.3)
      while(ball[1] > 24):
         self.peripherals.driveRobotForward(0.5, 0, 0)
         time.sleep(0.01)
      
      self.peripherals.driveRobotForward(0.0, 0, 0)
      self.peripherals.setVacuumMotorPWM(0.1)
      time.sleep(0.5)
      self.peripherals.setVacuumMotorPWM(0.3)
      time.sleep(1)

      self.peripherals.driveRobotForward(0.1, 0, 0)
      time.sleep(0.3)
      self.peripherals.driveRobotForward(0.2, 0, 0)
      time.sleep(0.3)
      self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)

      while(ball[1] > -10):
         self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
         self.peripherals.setVacuumMotorPWM(0.3)
         time.sleep(0.5)
      
      self.peripherals.driveRobotForward(0.0, 0, 0)
      time.sleep(0.5)
      self.peripherals.setVacuumMotorPWM(0.0)

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
               # print(dist_calc)
               ball[2] = dist_calc*10.5/50 - 7
               ball[3] = 0
               return
      dist_calc = dist(x,y, 0, 0)
      self.balls.append([x, y, dist_calc*10.5/50 - 7, 0])
      global_map.append([x + self.last_position[0], y + self.last_position[1], dist_calc*10.5/50 - 7, 0])
      
   def updateMap(self, delta_x, delta_y, delta_theta):
      mat = [[math.cos(delta_theta), -math.sin(delta_theta)], [math.sin(delta_theta), math.cos(delta_theta)]]
      for ball in self.balls:
         ball[0] = ball[0]*mat[0][0] + ball[1]*mat[0][1]
         ball[1] = ball[0]*mat[1][0] + ball[1]*mat[1][1]
         ball[0] -= delta_x
         ball[1] -= delta_y

mainMsgQueue   = queue.Queue()
ctrlMsgQueue   = queue.Queue()

task = ControlTask(mainMsgQueue, ctrlMsgQueue)

task.start()