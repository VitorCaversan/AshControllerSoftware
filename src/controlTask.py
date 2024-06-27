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

IR_GHOST_BALL_DIST = 180

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
      self.stuck_detect_thread = threading.Thread(target=self.runStuckDetection)

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
   
   def runStuckDetection(self):
      while True:
         self.peripherals.updateRobotStuck()
         time.sleep(0.5)

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
         
         time.sleep(0.4)

   def runRobot(self):
      print("running")
      # print(f"Sensor distance: {str(9.0)}, \nsteps: {str(self.peripherals.getLeftEncoderSteps())}, \ncollected balls: {str(self.peripherals.getCollectedBallsQty())}")
      # print(f"Ads value: {str(self.peripherals.getLowerBatteryADCVal())}, voltage: {str(self.peripherals.getLowerBatteryLvl())}\n")

      self.fsmInit()

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
      self.stuck_detect_thread.start()
      # self.static_obj_detector_th.start()
   
   def stop(self):
      self.peripherals.close()
      self.thread.join()
      self.odometry_thread.join()
      self.ball_detector_th.join()
      self.bluetooth_thread.join()
      self.stuck_detect_thread.join()
      # self.static_obj_detector_th.join()

   def safeExit(self, signum, frame):
      self.peripherals.close()
      self.odometry_thread.join()
      self.ball_detector_th.join()
      self.bluetooth_thread.join()
      self.stuck_detect_thread.join()
      # self.static_obj_detector_th.join()
      exit(1)
   
   def updateBtPeriodicMsg(self):
      self.btPeriodicMsg["sens_dist_left"] = round(self.peripherals.getLeftDistance(), 2)
      self.btPeriodicMsg["sens_dist_front"] = round(self.peripherals.getFrontDistance(), 2)
      self.btPeriodicMsg["sens_dist_right"] = round(self.peripherals.getRightDistance(), 2)
      self.btPeriodicMsg["sens_dist_back"] = round(self.peripherals.getBackDistance(), 2)
      self.btPeriodicMsg["battery_level"] = self.peripherals.getLowerBatteryLvl()
      self.btPeriodicMsg["balls_collected"] = self.peripherals.getCollectedBallsQty()
      self.btPeriodicMsg["balls_coordinates"] = []
      for ball in self.balls[:3]:
         self.btPeriodicMsg["balls_coordinates"].append([ball[0], ball[1]])
      self.btPeriodicMsg["robot_status"] = self.status

   def followWalls(self):
      print(f'')

      # Find the nearest wall in the robot
      # print('HERE')
      # dist = [self.peripherals.getFrontDistance(), self.peripherals.getLeftDistance(), self.peripherals.getRightDistance(), self.peripherals.getBackDistance()]
      # index = min(dist)
      # if dist.index(index) == 0:
      #    print('mais perto eh a frente')
      # elif index == 1:
      #    print('mais perto eh a esquerda')
      # elif index == 2:
      #    print('mais perto eh a direita')
      # elif index == 3:
      #    print('mais perto eh a tras')
      # pass

   def ballDetectorTh(self):
      # mean = 0
      i = 0
      while True:
         a = self.cam.capture_array("main")
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
            (a_t, ir_binary_img) = cv2.threshold(ir_gray_up, 230, 255,cv2.THRESH_BINARY)
            
            ir_blur_img = cv2.GaussianBlur(ir_binary_img, (17, 17), 0)
            ir_balls = cv2.HoughCircles(ir_blur_img, cv2.HOUGH_GRADIENT, 1.3, 10, param1=50, param2=20, minRadius=4, maxRadius=35)
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
               # cv2.waitKey(1)
            elif ((time.time() - self.time_findIR) > 7):
               self.baseIRPosition = [0, 0]
            cv2.imwrite('img_ir_gray.png', ir_gray_up)
            cv2.imwrite('img_ir.png', ir_blur_img)
            cv2.imwrite('img_ir_bin.png', ir_binary_img)
            
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
                        dist_ball *= 1400
                        horizontal_dist = 1.9*(i[0] - 480)/i[2] # 1.9cm is the radius of the ball
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
                        if dist_ball < 40:
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

   def staticObjDetectorTh(self):
      while True:
         a = self.cam.capture_array("main")
         b = self.cam1.capture_array("main")

         # a = cv2.resize(a, (960, 540))
         # b = cv2.resize(b, (960, 540))
         
         if a is None or b is None:
            continue
         print('AAAA')
         start = time.time()
         a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
         a = a[390:-57, :]
         b = cv2.cvtColor(b, cv2.COLOR_RGB2GRAY)
         b = b[447:, :]
         
         disp = self.stereo.compute(a, b)
         
         # Scaling down the disparity values and normalizing them 
         disp = (disp/16.0 - 5)/144
         disp = disp[:, 150:-150]
         (a_t, disp) = cv2.threshold(disp, 0.75, 0.99, cv2.THRESH_TOZERO)
         kernel = np.ones((11, 11), np.uint8)
         disp = cv2.erode(disp, kernel, iterations=1) 
         kernel = np.ones((3, 3), np.uint8)
         disp = cv2.erode(disp, kernel, iterations=1) 
         kernel = np.ones((19, 19), np.uint8) 
         disp = cv2.dilate(disp, kernel, iterations=1)
         # time.sleep(0.01)
         # disp = cv2.GaussianBlur(disp, (9, 9), 0)
         # disp = [d for d in disp if d > 0.65]
         disp_l = disp[:, :-330]
         disp_r = disp[:, 330:]
         sum_r = 0
         sum_l = 0
         for i in range(0, 90):
            for j in range(0, 330):
                  sum_r += disp_r[i][j]
                  sum_l += disp_l[i][j]
         
         # print("Sum left: ", sum_l)
         # print("Sum right: ", sum_r)
         if((sum_r + sum_l) > 300):
            if(sum_r > sum_l):
                  print("Object in right")
                  self.object_in_right = True
                  self.object_in_left = False
                  self.static_object_detected = True
            elif(sum_r < sum_l):
                  print("Object in left")
                  self.object_in_left = True
                  self.object_in_right = False
                  self.static_object_detected = True
         elif (self.peripherals.getFrontDistance() < 15):
            self.static_object_detected = True
         else:
            print("No obj Found")
            self.object_in_right = False
            self.object_in_left = False
            self.static_object_detected = False
            
         # time.sleep(0.01)
         print("Time: ", time.time() - start)

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
      
      self.actual_state = State.SEARCHING_BALLS
      self.next_state = State.SEARCHING_BALLS
      self.last_state = State.SEARCHING_BALLS

   def fsmRun(self):
      # Update booleans
      self.charge_complete = False           # Needs ADS
      self.opperation_complete = False       # How to define a complete operation??
      self.close_from_base = False           # Needs IR detection
      self.charger_connected = False         # Needs ADS
      self.battery_low = False               # Needs ADS     
      self.in_operation = False              # IDK
      # self.start_command_rcvd = False        # IDK
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
      elif(self.next_state == State.CONNECTED_TO_BASE):
         self.connectedToBase()
      elif(self.next_state == State.SEARCHING_BASE_WALL):
         self.searchingBaseFollowingWall()
      elif(self.next_state == State.CONNECTING_TO_BASE):
         self.connectingToBase()
      elif(self.next_state == State.FINDING_WALL):
         self.findingWall()
      elif(self.next_state == State.ROTATING_TILL_PARALLEL):
         self.rotateTillParallel()
      elif(self.next_state == State.SEARCHING_BASE_CAM):
         self.searchingBaseUsingCamera()
      elif(self.next_state == State.GOING_AFTER_BASE_CAM):
         self.goToBaseUsingCamera()
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
         print("Entering catching ball state")
         self.catchingBall()
      elif(self.next_state == State.BALL_STUCK):
         self.ballStuck()
      elif(self.next_state == State.WAITING_USER):
         self.waitForUser()
      elif(self.next_state == State.BASE_NOT_FOUND):
         self.baseNotFound()
      elif(self.next_state == State.ROTATE_TEST):
         self.rotateTest()

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
      
   def searchingBaseFollowingWall(self):
      # Entry
      if(self.actual_state != State.SEARCHING_BASE_WALL):
         self.actual_state = self.next_state
         self.status = "returning_to_base"
         
      # Do
      self.leftDistQueue.push(self.peripherals.getLeftDistance())
      self.moveParallelWall()
      self.findIR()
      
      # Exit
      if(self.baseIRPosition[0] != 0):
         self.next_state = State.GOING_AFTER_BASE_CAM
         self.last_state = self.actual_state
         self.leftDistQueue.reset()
         self.rotat_ctr_since_finding_wall = 0
      elif(self.rotat_ctr_since_finding_wall > 4):
         self.next_state = State.BASE_NOT_FOUND
         self.last_state = self.actual_state
         self.leftDistQueue.reset()
         self.rotat_ctr_since_finding_wall = 0
      elif(self.close_wall == True):
         self.lastRotationDir = 1
         self.next_state      = State.ROTATING_TILL_PARALLEL
         self.last_state      = self.actual_state
         self.leftDistQueue.reset()
         self.rotat_ctr_since_finding_wall += 1
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
         self.status = ""
   
   def findingWall(self):
      # Entry
      if(self.actual_state != State.FINDING_WALL):
         self.actual_state = self.next_state
         self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
      
      # Do
      self.findWall()
      
      # Exit
      if(self.close_wall == True):
         self.next_state      = State.ROTATING_TILL_PARALLEL
         self.last_state      = self.actual_state
         self.lastRotationDir = 1
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state

   def rotateTillParallel(self):
      # Entry
      if(self.actual_state != State.ROTATING_TILL_PARALLEL):
         self.actual_state = self.next_state
         self.is_rotating  = True
         self.peripherals.rotate(self.lastRotationDir, ((APPROX_PI/2) - 0.05), PWM_ROTATE)
         self.is_rotating  = False
      
      # Do
      self.leftDistQueue.push(self.peripherals.getLeftDistance())
      self.is_rotating = True
      self.peripherals.rotate(self.lastRotationDir, 0.08, PWM_ROTATE)
      time.sleep(0.2)
      self.is_rotating = False
      
      # Exit
      if(self.is_parallel_wall == True):
         self.next_state = State.SEARCHING_BASE_WALL
         self.last_state = self.actual_state
         self.leftDistQueue.reset()
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state

   def connectingToBase(self):
      # Entry
      if(self.actual_state != State.CONNECTING_TO_BASE):
         self.actual_state = self.next_state
         self.steerBase()
      
      # Do
      if (self.peripherals.getBackDistance() > 2):
         self.lastTimeFarFromBase = time.time()
      
      if (self.base_connected or ((time.time() - self.lastTimeFarFromBase > 1.0) and (self.peripherals.getBackDistance() < 2.0)) or self.peripherals.isRobotStuck()):
         self.base_connected = True
         self.peripherals.stopRobot()
         self.peripherals.setVacuumMotorPWM(0.0)

      # Exit
      if(self.base_connected == True):
         self.next_state          = State.CONNECTED_TO_BASE
         self.last_state          = self.actual_state
         self.baseIRPosition      = [0, 0]
         self.lastTimeFarFromBase = 0.0
         self.opperation_complete = True
         self.status = ""
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
         self.status = ""
   
   # Increase one state, moving to base
   def searchingBaseUsingCamera(self):
      # Entry
      if(self.actual_state != State.SEARCHING_BASE_CAM):
         self.findIR()
         time.sleep(2)
         if self.baseIRPosition[0] == 0:
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

   def goToBaseUsingCamera(self):
      if(self.actual_state != State.GOING_AFTER_BASE_CAM):
         self.actual_state = self.next_state
         self.status = "returning_to_base"
      
      # Do
      self.findIR()

      print(f"ir x: {self.baseIRPosition[0]}, ir y: {self.baseIRPosition[1]}")
      if (self.baseIRPosition[1] < IR_BALL_HEIGHT_FOR_BASE_CONNECT):
         self.peripherals.driveRobotForward(0.0, 0, 0)
         self.next_state = State.CONNECTING_TO_BASE
         self.search_for_IR = False
         self.last_state = self.actual_state
         self.status = ""
      elif abs(self.baseIRPosition[0] + IR_LED_DIST_FROM_BASE_CENTER_IN_PIXELS) > IR_CENTER_THRESHOLD_IN_PIXELS:
         print("will rotate")
         self.rotateToCenterBase()
      else:
         print("will set forward")
         self.peripherals.driveRobotForward(0.4, 0, 0)

      time.sleep(0.2)

      # Exit
      
      if(self.base_not_found == True):
         self.base_not_found = False
         self.next_state = State.FINDING_WALL
         self.last_state = self.actual_state
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
         self.status = ""

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
         self.opperation_complete = False
   
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
         self.starting_err_time = time.time()
         self.actual_state = self.next_state
      
      # Do
      
      # Exit
      if(time.time() - self.starting_err_time >= 15):
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
         self.status = ""
      
      # Do

      
      # Exit
      if(self.start_command_rcvd == True or self.start_schedule == True):
         self.enablePeripherals()
         self.start_command_rcvd == False
         self.next_state = State.SEARCHING_BALLS
         self.last_state = self.actual_state
   
   def searchingForBall(self):
      # Entry
      if(self.actual_state != State.SEARCHING_BALLS):
         self.actual_state = self.next_state
         self.peripherals.driveRobotForward(0.1, 0, 0)
         time.sleep(0.3)
         self.peripherals.driveRobotForward(0.2, 0, 0)
         time.sleep(0.3)
         self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
         time.sleep(0.3)
         self.status = "searching_for_balls"
      
      # Do
      self.moveInPattern()
      
      # Exit
      if(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
         self.status = ""
      elif(self.stop_command_rcvd == True or self.battery_low == True or self.load_full == True or self.end_schedule == True):
         self.next_state = State.SEARCHING_BASE_CAM
         self.last_state = self.actual_state
         self.status = ""
         self.stop_command_rcvd = False
      elif(self.pause_command_rcvd == True):
         self.next_state = State.PAUSED
         self.last_state = self.actual_state
         self.pause_command_rcvd = False
         self.status = ""
      elif(self.static_object_detected == True or self.peripherals.isRobotStuck()):
         self.next_state = State.AVOIDING_STATIC_OBJECT
         self.last_state = self.actual_state
         self.status = ""
      elif(self.ball_detected == True):
         self.next_state = State.CATCHING_BALL
         self.last_state = self.actual_state
         self.stop_ball_search = True
         self.status = ""
      
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
         self.actual_state = self.next_state
      
      # Do
      self.moveAroundObject()
      
      # Exit
      if(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
      elif(self.static_object_avoided == True):
         self.next_state = State.SEARCHING_BALLS
         self.last_state = self.actual_state
      elif(self.pause_command_rcvd == True):
         self.next_state = State.PAUSED
         self.last_state = self.actual_state
         self.pause_command_rcvd = False
   
   def robotPaused(self):
      if(self.actual_state != State.PAUSED):
         self.stopMotors()
         self.actual_state = self.next_state
         self.status = "paused"
      
      # Do
      
      # Exit
      if((self.resume_command_rcvd == True) or (self.start_command_rcvd == True)):
         self.next_state = self.last_state
         self.last_state = self.actual_state
         self.resume_command_rcvd = False
         self.start_command_rcvd == False
         self.status = ""
   
   def catchingBall(self):
      if(self.actual_state != State.CATCHING_BALL):
         self.status = "collecting_balls"
         self.actual_state = self.next_state
      
      # Do
      self.approachBall()

      # Due to too much ghost balls
      self.balls = []
      # Due to not 100% proof logic
      self.ball_caught = True
      
      # Exit
      if(self.ball_caught == True):
         #self.increaseSpeed()
         #self.reduceVacuumPower()
         self.stop_ball_search = False
         self.next_state = State.SEARCHING_BALLS
         self.last_state = self.actual_state
         self.status = ""
      elif(self.robot_running_encoder == True and self.robot_running_imu == False):
         self.next_state = State.ROBOT_STUCK
         self.last_state = self.actual_state
         self.status = ""
      elif(self.pause_command_rcvd == True):
         self.next_state = State.PAUSED
         self.last_state = self.actual_state
         self.pause_command_rcvd = False
      elif(self.front_ir_detected_but_not_end == True):
         self.next_state = State.BALL_STUCK
         self.last_state = self.actual_state
         self.status = ""
      
   def ballStuck(self):
      if(self.actual_state != State.BALL_STUCK):
         self.stopMotors()
         self.starting_err_time = time.time()
         self.actual_state = self.next_state
      
      # Do
      self.sendWarningUser("ball_stuck")

      # Exit
      if(self.front_ir_detected_but_not_end == True):
         self.next_state = State.SEARCHING_BALLS
         self.sendWarningUser("")
         self.last_state = self.actual_state
      elif(time.time() - self.starting_err_time > 3):
         self.next_state = State.WAITING_USER
         self.last_state = self.actual_state

   def waitForUser(self):
      if(self.actual_state != State.WAITING_USER):
         self.stopMotors()
         self.actual_state = self.next_state
      
      # Do
      
      # Exit
      if (self.start_command_rcvd == True):
         self.next_state = State.INIT
         self.last_state = self.actual_state
         # start_command_rcvd must remain True so that the robot can pass through the
         # WAITING_FOR_CMD_OR_SCHEDULE state with only 1 click of the start button
         # self.start_command_rcvd = False
   
   def robotStuck(self):
      if(self.actual_state != State.ROBOT_STUCK):
         self.stopMotors()
         self.starting_err_time = time.time()
         self.actual_state = self.next_state
      
      # Do
      self.sendWarningUser("robot_stuck")

      # Exit
      if(self.robot_running_imu == True and self.robot_running_encoder == True):
         self.next_state = self.last_state
         self.sendWarningUser("")
         self.last_state = self.actual_state
      elif(time.time() - self.starting_err_time > 3):
         self.next_state = State.WAITING_USER
         self.last_state = self.actual_state

   def baseNotFound(self):
      # Entry
      if(self.actual_state != State.BASE_NOT_FOUND):
         self.starting_err_time = time.time()
         self.actual_state = self.next_state
      
      # Do
      self.sendWarningUser("base_not_found")

      # Exit
      if(time.time() - self.starting_err_time >= 1):
         self.sendWarningUser("")
         self.next_state = State.PAUSED
         self.last_state = self.actual_state

   def findWall(self):
      # It is just moving forward until it finds a wall
      return

   # Takes a populated array  of distances to the left wall.
   # If the average distance is less than DIST_TOO_CLOSE_IN_CM cm, the robot rotates to the right a small amount,
   # moves a bit forward and then rotates to the left the same amount.
   # If the average distance is greater than 40 cm, the robot rotates to the left a small amount,
   # moves a bit forward and then rotates to the right the same amount.
   # If the average distance is between DIST_TOO_CLOSE_IN_CM and (DIST_TOO_CLOSE_IN_CM + 10) cm, the robot moves forward.
   def moveParallelWall(self):
      if (self.leftDistQueue.getLength() < LEFT_DIST_QUEUE_SIZE):
         return
      
      avgDist = self.leftDistQueue.getAvg()

      if (avgDist < DIST_TOO_CLOSE_IN_CM):
         self.is_rotating = True
         self.peripherals.rotate(1, 0.1, PWM_ROTATE)
         self.is_rotating = False
         time.sleep(0.2)
         self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
         time.sleep(0.2)
         self.is_rotating = True
         self.peripherals.rotate(0, 0.1, PWM_ROTATE)
         self.is_rotating = False
      elif (avgDist > (DIST_TOO_CLOSE_IN_CM + 10)):
         self.is_rotating = True
         self.peripherals.rotate(0, 0.1, PWM_ROTATE)
         self.is_rotating = False
         time.sleep(0.2)
         self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
         time.sleep(0.2)
         self.is_rotating = True
         self.peripherals.rotate(1, 0.1, PWM_ROTATE)
         self.is_rotating = False
      else:
         self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
         time.sleep(0.2)
         self.peripherals.driveRobotForward(0.5, 0, 0)
         time.sleep(0.2)
         self.peripherals.driveRobotForward(0.7, 0, 0)
      
      return

   # Rotates the robot, aligns with the center of the base and parks the robot backwards
   def steerBase(self):
      robot_ang_rel_to_base = math.atan2(self.last_position[1], self.last_position[0])
      if (abs(robot_ang_rel_to_base) > 0.523):
         robot_ang_rel_to_base = 0.523

      self.is_rotating = True
      self.peripherals.rotate(0, APPROX_PI, PWM_ROTATE) # Rotates 180 degrees
      self.is_rotating = False

      # Moves backwards in a straight line for time_to_go seconds
      desired_dist_to_go_bkwr = 0.075
      desired_pwm = PWM_ROTATE
      desired_pwm = (desired_pwm * (PWM_FOR_MAX_SPEED - PWM_FOR_MIN_SPEED)) + PWM_FOR_MIN_SPEED
      m_per_s = ((MAX_MOTOR_RMP * desired_pwm) * WHEEL_CIRCUMFERENCE_IN_M) / 60
      time_to_go = desired_dist_to_go_bkwr / m_per_s
      self.peripherals.driveRobotBackward(desired_pwm, 0, 0)
      time.sleep(time_to_go)

      if (self.peripherals.getBackDistance() > 2):
         self.is_rotating = True
         if (robot_ang_rel_to_base < 0):
            self.peripherals.rotate(1, abs(robot_ang_rel_to_base), PWM_ROTATE)
         else:
            self.peripherals.rotate(0, abs(robot_ang_rel_to_base), PWM_ROTATE)
         self.is_rotating = False
      
      self.peripherals.driveRobotBackward(0.3, 0, 0)

      return
   
   def findIR(self):
      if(self.search_for_IR == False):
         print("finding ir")
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
      self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)

      if self.peripherals.getFrontDistance() < 30:
         self.is_rotating = True
         self.stop_ball_search = True
         self.peripherals.rotate(1, APPROX_PI / 2, PWM_ROTATE)
         self.is_rotating = False
         self.stop_ball_search = False

         self.peripherals.driveRobotForward(0.1, 0, 0)
         time.sleep(0.3)
         self.peripherals.driveRobotForward(0.2, 0, 0)
         time.sleep(0.3)
         self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
         time.sleep(0.3)

      if self.balls != []:
         self.next_state = State.CATCHING_BALL

      return

   # Takes the robot position from odometry, calculates the angle of the base relative to the robot,
   # compares it with the angle of the robot and rotates the robot to align it with the base
   def rotateInDirectOfBase(self):
      self.peripherals.driveRobotForward(0.0, 0, 0)

      base_angle = math.atan2(self.last_position[1], self.last_position[0])
      base_angle_rel_to_robot = APPROX_PI - base_angle
      robot_angle = self.last_position[2]

      if (robot_angle < 0):
         robot_angle += 2*APPROX_PI

      delta_angle = robot_angle - base_angle_rel_to_robot

      # Rotates a little bit less than needed
      self.is_rotating = True
      if delta_angle < 0:
         self.lastRotationDir = 0
         self.peripherals.rotate(self.lastRotationDir, ((-delta_angle) - 0.2), PWM_ROTATE)
      else:
         self.lastRotationDir = 1
         self.peripherals.rotate(self.lastRotationDir, (delta_angle - 0.2), PWM_ROTATE)
      self.is_rotating = False

   # Rotates the robot according to the baseIRPosition found, to lign it up with the base
   def rotateToCenterBase(self):
      self.is_rotating = True
      if ((self.baseIRPosition[0] + IR_LED_DIST_FROM_BASE_CENTER_IN_PIXELS) > IR_CENTER_THRESHOLD_IN_PIXELS):
         self.peripherals.rotate(1, 0.02, PWM_ROTATE - 0.05)
      elif ((self.baseIRPosition[0] + IR_LED_DIST_FROM_BASE_CENTER_IN_PIXELS) < -IR_CENTER_THRESHOLD_IN_PIXELS):
         self.peripherals.rotate(0, 0.02, PWM_ROTATE - 0.05)
      self.is_rotating = False

      time.sleep(0.5)

      return

   # Improve this
   def moveAroundObject(self):
      self.static_object_avoided = False
      while(self.peripherals.getFrontDistance() < 15 or self.peripherals.isRobotStuck()):
         self.peripherals.driveRobotBackward(PWM_FORWARD-0.1, 0, 0)
         time.sleep(1)
      self.is_rotating = True
      last_dist = 0
      self.peripherals.stopRobot()
      time.sleep(0.2)
      self.peripherals.rotate(1, APPROX_PI / 2, PWM_ROTATE)
      # while(self.peripherals.getFrontDistance() < 20):
      #    self.peripherals.rotate(0,0.1,PWM_ROTATE-0.1)
      #    last_dist = self.peripherals.getFrontDistance()
      # d2 = last_dist*math.sin(math.pi/12)
      # d3 = last_dist*math.cos(math.pi/12)
      # # print(d2)
      # # print(d3)
      # d4 = 12 - d2
      # if(d4 > 0):
      #    theta = math.atan(d4/d3)
      #    self.peripherals.rotate(0,theta*2,PWM_ROTATE-0.1)
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

      avgDistribution = (6*angle/10)
      if angle > 0.75:
         avgDistribution = (7*angle/10)
      # ball[0] = ball[0] + 3
      
      time.sleep(1)
      while(ball[0] > 1 or ball[0] < -1):
         print(f"ball zero: {ball[0]}, angle: {angle}, angle sum: {angle_sum}")
         self.is_rotating = True
         if(ball[0] > 0):
            self.peripherals.rotate(1, 0.01, PWM_ROTATE-0.05)
         else:
            self.peripherals.rotate(0, 0.01, PWM_ROTATE-0.05)
         self.is_rotating = False
         angle_sum += 0.01
         if ((angle_sum > (angle - avgDistribution)) and ball[0] < -1):
            break
         if ((angle_sum > (angle - avgDistribution)) and ball[0] > 1):
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
      time.sleep(0.5)

      self.peripherals.driveRobotForward(0.1, 0, 0)
      time.sleep(0.3)
      self.peripherals.driveRobotForward(0.2, 0, 0)
      time.sleep(0.3)
      self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)

      while(ball[1] > -20):
         self.peripherals.driveRobotForward(PWM_FORWARD, 0, 0)
         self.peripherals.setVacuumMotorPWM(0.3)
         time.sleep(0.5)
      
      self.peripherals.driveRobotForward(0.0, 0, 0)
      time.sleep(0.5)
      self.peripherals.setVacuumMotorPWM(0.0)

      return

   def sendWarningUser(self, error:str):
      self.btPeriodicMsg["robot_error"] = error

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
      # self.global_map.sort(key=closest)
      
   def updateMap(self, delta_x, delta_y, delta_theta):
      mat = [[math.cos(delta_theta), -math.sin(delta_theta)], [math.sin(delta_theta), math.cos(delta_theta)]]
      for ball in self.balls:
         ball[0] = ball[0]*mat[0][0] + ball[1]*mat[0][1]
         ball[1] = ball[0]*mat[1][0] + ball[1]*mat[1][1]
         ball[0] -= delta_x
         ball[1] -= delta_y

      # self.balls.sort(key=closest())

   def rotateTest(self):
      # print("Entering rotateTest")
      
      #angle = math.atan(ball[0]/ball[1])
      #angle = abs(angle)
      # angle_sum = 0

      # print(
      #    f"""
      #    Camera trace
      #    Coord: {ball}
      #    Angle: {angle}
      #    """
      # )
      self.findIR()
      # time.sleep(1)
      # while(ball[0] > 1 or ball[0] < -1):
      #    print(f"ball zero: {ball[0]}")
      #    self.is_rotating = True
      #    if(ball[0] > 0):
      #       self.peripherals.rotate(1, 0.01, 0.25)
      #    else:
      #       self.peripherals.rotate(0, 0.01, 0.25)
      #    self.is_rotating = False
      #    angle_sum += 0.01
      #    if(angle_sum > (angle - (angle/10))):
      #       break
      #    time.sleep(0.1)

      # print(self.balls)
      # self.balls = []
      # print("Restarting test algorithm")
      # time.sleep(10)
