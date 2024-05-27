from gpiozero import Robot, Motor
import time

MOTOR_STEPS_PER_TURN = 872
ROBOT_RADIUS_FROM_CENTER_IN_M = 0.1
WHEEL_DIAMETER_IN_M = 0.068
WHEEL_CIRCUMFERENCE_IN_M = 3.141592 * WHEEL_DIAMETER_IN_M
MOTOR_STEPS_PER_M = MOTOR_STEPS_PER_TURN / WHEEL_CIRCUMFERENCE_IN_M

robot = Robot(left=(Motor(26, 19)), right=(Motor(21, 20)))

def driveRobotForward(speed: float, curveLeftRate: float, curveRightRate: float):
    if ((curveLeftRate < 0 or curveLeftRate > 1) or
        (curveRightRate < 0 or curveRightRate > 1) or
        (speed < 0 or speed > 1)):
        print("Invalid speed values")
        return
      
    robot.forward(speed=speed, curve_left=curveLeftRate, curve_right=curveRightRate)
    
   # Drive the robot backward by running both motors backward.
   # Left and right relative to the robot it
def driveRobotBackward(speed: float, curveLeftRate: float, curveRightRate: float):
    if ((curveLeftRate < 0 or curveLeftRate > 1) or
        (curveRightRate < 0 or curveRightRate > 1) or
        (speed < 0 or speed > 1)):
        print("Invalid speed values")
        return
      
    robot.backward(speed=speed, curve_left=curveLeftRate, curve_right=curveRightRate)

def rotate(direction: int, angle: float):
    initialSteps: int = 0
    # encoderSteps = {0: encoderRight.steps, 1: encoderLeft.steps}

    if (direction == 0):
    #     initialSteps = encoderRight.steps
        robot.left()
    elif (direction == 1):
    #     initialSteps = self.encoderLeft.steps
        robot.right()
    # else:
    #     print("Invalid direction")
    #     return
    
    archSize = angle * ROBOT_RADIUS_FROM_CENTER_IN_M
    stepsToTurn = archSize * MOTOR_STEPS_PER_M

    while (abs(initialSteps) < stepsToTurn):
        time.sleep(0.01)
    
    robot.stop()

    return



driveRobotBackward(1, 0,0)
time.sleep(5)

rotate(1,1)
time.sleep(5)

driveRobotBackward(1, 0,0)
time.sleep(5)