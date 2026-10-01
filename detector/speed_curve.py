import time
import math

MAX_SPEED = 100
LOOP_TIME = 0.01

def curve(speed, setpoint, rate):
    if(speed > setpoint):
        speed -= rate
        if(speed < 0):
            speed = 0
    elif(speed < setpoint): 
        speed += rate
        if(speed > MAX_SPEED):
            speed = MAX_SPEED
    return speed


def curve_time(speed, setpoint, duration):
    steps = duration/LOOP_TIME
    rate = math.fabs(setpoint - speed)/steps
    for i in range(0, int(steps)):
        speed = curve(speed, setpoint, rate)
        print(speed)
        time.sleep(LOOP_TIME)
    return speed

speed = 0

print("curve by time")
speed = curve_time(speed, 50, 1)
speed = curve_time(speed, 50, 1)
speed = curve_time(speed, 0, 1)
speed = curve_time(speed, 0, 0.5)
speed = curve_time(speed, 100, 1)
speed = curve_time(speed, 25, 3)
speed = curve_time(speed, 75, 2)