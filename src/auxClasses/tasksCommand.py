HOUR = 0
MINUTE = 1
SECOND = 2

class TasksCommand:
   def __init__(self):
      self.startTasksNow = False
      self.timeToStartTasks = ["00", "00", "00"]

   def setStartTasksNow(self, startTasksNow):
      self.startTasksNow = startTasksNow
   
   def mustStartNow(self):
      return self.startTasksNow
   
   def setTimeToStartTasks(self, timeToStartTasks):
      parsedTime = timeToStartTasks.split(":")
      self.timeToStartTasks[HOUR] = parsedTime[HOUR]
      self.timeToStartTasks[MINUTE] = parsedTime[MINUTE]
      self.timeToStartTasks[SECOND] = parsedTime[SECOND]

   def getTimeToStartTasks(self):
      return self.timeToStartTasks

   def haveTimeToStart(self):
      return ((self.timeToStartTasks[HOUR] != "00") or
              (self.timeToStartTasks[MINUTE] != "00") or
              (self.timeToStartTasks[SECOND] != "00"))

   def gettimeToStartTasks(self):
      return self.timeToStartTasks
   
   def __str__(self) -> str:
      if self.startTasksNow:
         return "Start tasks now"
      elif self.haveTimeToStart():
         return "Start tasks at " + self.timeToStartTasks[HOUR] + ":" + self.timeToStartTasks[MINUTE] + ":" + self.timeToStartTasks[SECOND]
      else:
         return "No tasks to start"