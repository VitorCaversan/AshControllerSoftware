from picamera2 import Picamera2, Preview
import time
import cv2
import numpy as np
import math

def nothing(x):
    pass

balls = []

def convert(dist, theta):
    x = dist*math.sin(theta)
    y = dist*math.cos(theta)
    return (x, y)

def verifier():
    for i in range(0, len(balls)):
        if(i > len(balls)):
            break
        if(balls[i][1] < 0):
            balls.pop(i)
            i -= 1
        elif(balls[i][1] > 50):
            balls[i][3] += 1
            if(balls[i][3] > 20):
                balls.pop(i)
                i -= 1

dist = lambda x1,y1,x2,y2: math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def addBall(x, y):
    for ball in balls:
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
    balls.append([x, y, dist_calc*10.5/50 - 7, 0])
    
def updateMap(delta_x, delta_y):
    for ball in balls:
        ball[0] += delta_x
        ball[1] += delta_y

cam = Picamera2(0)
cfg = cam.create_preview_configuration(main={'size': (1920, 1080)})
cam.configure(cfg)
# cam.set_controls({"FrameRate": 5})
cam.resolution = (1920, 1080)
cam.framerate = 10
prev_circle = None


cam.start()
cam1.start()
stereo = cv2.StereoBM.create()
mean = 0
i = 0
while True:
    a = cam.capture_array("main")
    a = cv2.cvtColor(a, cv2.COLOR_BGR2RGB)
    a = cv2.resize(a, (960, 540))
    
    updateMap(0, 0)

    if a is None:
        continue

    a_grey = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY)
    a_grey_up = a_grey[300:, :]
    a_grey = a_grey[300:, :]
    a_grey = cv2.normalize(a_grey, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    (a_t, threshInv) = cv2.threshold(a_grey_up, 200, 255,cv2.THRESH_BINARY)
    a_blur = cv2.GaussianBlur(a_grey, (17, 17), 0)
    circles = cv2.HoughCircles(a_blur, cv2.HOUGH_GRADIENT, 1.2, 10, param1=100, param2=35, minRadius=13, maxRadius=46)
    # print(circles)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        ball_added = False
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
                # print("dist: ", i[2], " pixel")
                # print("dist: ", dist_ball, " cm")
                # print("hor dist: ", horizontal_dist, " cm")
                # print("angle with camera: ", theta*180/math.pi)
                # print("center dist: ", dist_center_ball, " cm")
                # print("angle with center: ", theta_center*180/math.pi)
                x_ball, y_ball = convert(dist_center_ball, theta_center)
                if dist_ball < 100:
                    addBall(x_ball, y_ball)

    # verifier()
    print(balls)
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
