
from picamera2 import Picamera2, Preview
import time
import cv2
import numpy as np

import matplotlib
matplotlib.use('GTK3Agg')
from matplotlib import pyplot as plt

def nothing(x):
    pass


cam = Picamera2(0)
cam1 = Picamera2(1)
cfg = cam.create_preview_configuration(main={'size': (960, 540)})
cfg1 = cam1.create_preview_configuration(main={'size': (960, 540)})
cam.configure(cfg)
cam1.configure(cfg1)
# cam.set_controls({"FrameRate": 5})
# cam1.set_controls({"FrameRate": 5})
cam.resolution = (960, 540)
cam1.resolution = (960, 540)
cam.framerate = 2
cam1.framerate = 2
#cam.start_preview(Preview.QT)
#cam1.start_preview(Preview.QT)

cam.start()
cam1.start()
# stereo = cv2.StereoBM.create()
stereo = cv2.StereoSGBM_create()
stereo.setNumDisparities(160)
stereo.setBlockSize(9)
# stereo.setPreFilterType(preFilterType)
# stereo.setPreFilterSize(preFilterSize)
stereo.setPreFilterCap(5)
# stereo.setTextureThreshold(textureThreshold)
stereo.setUniquenessRatio(5)
stereo.setSpeckleRange(18)
stereo.setSpeckleWindowSize(7)
stereo.setDisp12MaxDiff(0)
stereo.setMinDisparity(110)
# cv_file = cv2.FileStorage("./stereo_rectify_maps.xml", cv2.FILE_STORAGE_READ)
# Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
# Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
# Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
# Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
# cv_file.release()

while True:
    a = cam.capture_array("main")
    b = cam1.capture_array("main")
    
    print("a")
    if a is None or b is None:
        continue
    start = time.time()
    a = cv2.cvtColor(a, cv2.COLOR_BGR2GRAY)
    a = a[390:-57, :]
    b = cv2.cvtColor(b, cv2.COLOR_RGB2GRAY)
    b = b[447:, :]
    # time.sleep(0.01)
    # a = cv2.normalize(a, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    # b = cv2.normalize(b, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    
    # cv2.imshow("b", b)
    # cv2.waitKey(1)
    # cv2.imshow("a", a)
    # cv2.waitKey(1)
    # print(len(a))
    # Left_nice= cv2.remap(a,
    #           Left_Stereo_Map_x,
    #           Left_Stereo_Map_y,
    #           cv2.INTER_LANCZOS4,
    #           cv2.BORDER_CONSTANT,
    #           0)
     
    # Applying stereo image rectification on the right image
    # Right_nice= cv2.remap(b,
    #           Right_Stereo_Map_x,
    #           Right_Stereo_Map_y,
    #           cv2.INTER_LANCZOS4,
    #           cv2.BORDER_CONSTANT,
    #           0)

    # disparity = stereo.compute(a,b)
    
    disp = stereo.compute(a, b)
    
    # time.sleep(0.01)
    # disp = cv2.normalize(disp,0,255,cv2.NORM_MINMAX)
    
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
    
    print("Sum left: ", sum_l)
    print("Sum right: ", sum_r)
    if((sum_r + sum_l) > 300):
        if(sum_r > sum_l):
            print("Object in right")
        elif(sum_r < sum_l):
            print("Object in left")
    else:
        print("No obj Found")
    # time.sleep(0.01)
    print("Time: ", time.time() - start)
    # time.sleep(0.1 - (time.time() - start))
    
    
    # print (max(map(max, disp)))

    # a_blur = cv2.GaussianBlur(a, (9, 9), 0)
    # edged = cv2.Canny(a, 50, 120) 
    # contours, hierarchy = cv2.findContours(edged, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    # cv2.drawContours(a, contours, -1, 255, -1) 

    # plt.imshow(disparity,'gray')
    # cv2.imshow("disp_im", disp)
    # cv2.waitKey(1)
    # cv2.imshow("mask", a)

    
    # if cv2.waitKey(1) == 27:
    #   break
    # plt.show()
# cv2.imshow("a", cam.capture_array())
time.sleep(10)
# cam.end_preview()