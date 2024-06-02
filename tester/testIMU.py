


import adafruit_icm20x as IMU
import board
import time

imu            = IMU.ICM20948(board.I2C(), 0x68)

constant_error = [-0.012669, 0.007080, -0.006842]

acc_acceleration = [0, 0, 0]
acc_gyro = [0, 0, 0]
i = 0
while(1):
    acc_acceleration[0] = imu.acceleration[0]
    acc_acceleration[1] = imu.acceleration[1]
    acc_acceleration[2] = imu.acceleration[2]

    acc_gyro[0] = (acc_gyro[0]*i + imu.gyro[0] - constant_error[0])/(i+1)
    acc_gyro[1] = (acc_gyro[1]*i + imu.gyro[1] - constant_error[1])/(i+1)
    acc_gyro[2] = (acc_gyro[2]*i + imu.gyro[2] - constant_error[2])/(i+1)

    # print(acc_acceleration)
    

    i += 1
    print(acc_gyro[0])
    print(acc_gyro[1])
    print(acc_gyro[2])
    time.sleep(0.1)