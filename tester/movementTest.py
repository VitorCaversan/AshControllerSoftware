from gpiozero import Robot, Motor


def driveRobotForward(speed: float, curveLeftRate: float, curveRightRate: float):
    if ((curveLeftRate < 0 or curveLeftRate > 1) or
        (curveRightRate < 0 or curveRightRate > 1) or
        (speed < 0 or speed > 1)):
        print("Invalid speed values")
        return
      
    robot.forward(speed=speed, curve_left=curveLeftRate, curve_right=curveRightRate)
    
   # Drive the robot backward by running both motors backward.
   # Left and right relative to the robot it
def driveRobotBackward(, speed: float, curveLeftRate: float, curveRightRate: float):
    if ((curveLeftRate < 0 or curveLeftRate > 1) or
        (curveRightRate < 0 or curveRightRate > 1) or
        (speed < 0 or speed > 1)):
        print("Invalid speed values")
        return
      
    robot.backward(speed=speed, curve_left=curveLeftRate, curve_right=curveRightRate)