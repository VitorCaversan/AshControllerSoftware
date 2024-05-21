import schedule
import queue
from controlTask import ControlTask

class TaskScheduler:
   def __init__(self, mainQueue: queue.Queue, ctrlQueue: queue.Queue):
      self.currentTask = None
      self.controlTask = ControlTask(mainQueue=mainQueue, ctrlQueue=ctrlQueue)
   
   def runRoutineNow(self):
      print("Running routine now")
      self.controlTask.start()

   def runRoutineAt(self, hour, min, sec):
      schedule.every().day.at(f"{hour}:{min}:{sec}").do(self.runRoutineNow)

   def stopRoutine(self):
      self.controlTask.stop()