from gpiozero import DistanceSensor
import time

# leftDistSens     = DistanceSensor(echo=22, trigger=17, threshold_distance=0.15)
frontDistSens   = DistanceSensor(echo=27, trigger=17, threshold_distance=0.15)
# rightDistSens   = DistanceSensor(echo=10, trigger=17, threshold_distance=0.15)
# backDistSens    = DistanceSensor(echo=9, trigger=17, threshold_distance=0.15)



# def getLeftDistance() -> float:
#    return leftDistSens.distance * 100.0

def getFrontDistance() -> float:
   return frontDistSens.distance * 100.0

# def getRightDistance() -> float:
#    return rightDistSens.distance * 100.0

# def getBackDistance() -> float:
#    return backDistSens.distance * 100.0


i=0
while(i<100):
    # print("Dist back: ",  getBackDistance())
    # print("Dist right: ",  getRightDistance())
    # print("Dist back: ",  getBackDistance())
    print("Dist front: ",  getFrontDistance())
    time.sleep(0.5)
    i+=1



# leftDistSens.close()
frontDistSens.close()
# rightDistSens.close()
# backDistSens.close()