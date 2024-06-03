import time
import math

LEFT_MOTOR_PPR = 872
RIGHT_MOTOR_PPR = 872

# The standart unit for distance is meters
WHEEL_DISTANCE = 0.24
WHEEL_DIAMETER = 0.068
WHEEL_CIRCUMFERENCE = 3.141592 * WHEEL_DIAMETER

class EncoderSensorMnger:
    def __init__(self) -> None:
        self.position_x = 0.0
        self.position_y = 0.0
        self.rotation_z = 0.0

        self.leftLastEncoder = 0.0
        self.RightLastEncoder = 0.0

        self.lastLeftMotorSteps = 0.0
        self.lastRightMotorSteps = 0.0

    def routine(self, encoderLeft, encoderRight):
        # 1. Calculate the encoder count difference
        leftMotorStepsDiff  = encoderLeft.steps - self.lastLeftMotorSteps
        rightMotorStepsDiff = encoderRight.steps - self.lastRightMotorSteps
        print("Encoder left: ", encoderLeft.steps)
        print("Encoder right: ", encoderRight.steps)
        print("Encoder left diff: ", leftMotorStepsDiff)
        print("Encoder right diff: ", rightMotorStepsDiff)
        self.lastLeftMotorSteps  = encoderLeft.steps
        self.lastRightMotorSteps = encoderRight.steps

        # 2. Calculate dl and dr
        dl = leftMotorStepsDiff * WHEEL_CIRCUMFERENCE / LEFT_MOTOR_PPR
        dr = rightMotorStepsDiff * WHEEL_CIRCUMFERENCE / RIGHT_MOTOR_PPR

        # 3. Calculate delta angle
        delta_angle = (dr - dl) / (2 * WHEEL_DISTANCE)

        # 4. Calculate delta position
        delta_position = (dr + dl) / 2

        # 5. Calculate delta_x and delta_y
        delta_x = delta_position * math.cos(delta_angle)
        delta_y = delta_position * math.sin(delta_angle)

        # 6. Somatory
        self.position_x += delta_x
        self.position_y += delta_y
        self.rotation_z += delta_angle

        print(f"Current position: {self.position_x} {self.position_y} {self.rotation_z}")
        
    def getLocation():
        # Return the position X, Y and the rotation in the 
        # Z axis of the robot related to the base coordinate system
        # Ouput format: (X, Y, Z)
        # X: coordinate in the X axis related to the base coordinate system
        # Y: coordinate in the Y axis related to the base coordinate system
        # Z: angle in radian of the robot
        pass