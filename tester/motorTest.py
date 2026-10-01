from gpiozero import PWMOutputDevice
import time

vacuumMotor = PWMOutputDevice(pin=18, frequency=1500)

MAX_VALUE = 100

def setVacuumMotorPWM(pwm: float):
    if (pwm >= 0) and (pwm <= 1):
        vacuumMotor.blink(on_time=(0.01*pwm), off_time=(0.01*(1-pwm)))

setVacuumMotorPWM(0.3)
time.sleep(1)
# setVacuumMotorPWM(0.2)
# time.sleep(1)
# setVacuumMotorPWM(0.3)
# time.sleep(1)
# setVacuumMotorPWM(0.3)
# time.sleep(10)

# setVacuumMotorPWM(0.2)
time.sleep(5)
setVacuumMotorPWM(0)