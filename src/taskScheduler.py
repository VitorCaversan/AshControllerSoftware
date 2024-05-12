import schedule
from peripheralsTask import PeripheralsTask

class TaskScheduler:
   def __init__(self):
      self.currentTask = None
      self.peripheralsTask = PeripheralsTask()
   
   def runRoutineNow(self):
      print("Running routine now")
      self.peripheralsTask.start()

   def runRoutineAt(self, hour, min, sec):
      schedule.every().day.at(f"{hour}:{min}:{sec}").do(self.runRoutineNow)

   def stopRoutine(self):
      self.peripheralsTask.stop()