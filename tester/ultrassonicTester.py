from gpiozero import DistanceSensor, DigitalInputDevice
import time


leftDistSens     = DistanceSensor(echo=27, trigger=17, threshold_distance=0.15)
frontDistSens   = DistanceSensor(echo=9, trigger=13, threshold_distance=0.15)
rightDistSens   = DistanceSensor(echo=22, trigger=11, threshold_distance=0.15)
backDistSens    = DistanceSensor(echo=10, trigger=0, threshold_distance=0.15)

hallEffectSens = DigitalInputDevice(pin=23, pull_up=None, active_state=False)


def getLeftDistance() -> float:
   return leftDistSens.distance * 100.0

def getFrontDistance() -> float:
   return frontDistSens.distance * 100.0
    # return 0

def getRightDistance() -> float:
   return rightDistSens.distance * 100.0
    # return 0

def getBackDistance() -> float:
   return backDistSens.distance * 100.0
    # return 0


i=0
while True:
    # print("Dist left: ",  getLeftDistance())
    # print("Dist right: ",  getRightDistance())
    # print("Dist back: ",  getBackDistance())
    # print("Dist front: ",  getFrontDistance())
    print(f"Hall sensor: {hallEffectSens.is_active}")
    time.sleep(0.5)
    i+=1



leftDistSens.close()
frontDistSens.close()
rightDistSens.close()
backDistSens.close()