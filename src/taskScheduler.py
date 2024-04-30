import schedule
from motorTask import MotorTask

class TaskScheduler:
   def __init__(self):
      self.currentTask = None
      self.motorTask = MotorTask()
   
   def runRoutineNow(self):
      print("Running routine now")
      self.motorTask.start()

   def runRoutineAt(self, hour, min, sec):
      schedule.every().day.at(f"{hour}:{min}:{sec}").do(self.runRoutineNow)

   def stopRoutine(self):
      self.motorTask.stop()