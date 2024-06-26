import json
from enum import Enum
from auxClasses.tasksCommand import TasksCommand

# A class containing info parsed from a received Bluetooth message such as:
# {
#   "robot_command": "start", # options: start, return_to_base, pause, resume, schedule
#   "schedule": {
#     # Only if robot_command is "schedule"
#     "start_time": "10:00:00",
#     "end_time": "12:00:00"
#   }
# }


# A class containing all info needed to handle task scheduling
class RxBluetoothMsg:
   def __init__(self):
      self.msg = ""
      self.robot_command = ""
      self.tasksCommand = TasksCommand()

   def parseMsg(self, msg, firstMsgReceived: bool):
      self.msg = msg
      parsedMsg = json.loads(self.msg)
      self.robot_command = parsedMsg["robot_command"]


      if self.robot_command == "start" and firstMsgReceived == False:
         self.tasksCommand.setStartTasksNow(True)
      elif self.robot_command == "schedule" and firstMsgReceived == False:
         self.tasksCommand.setStartTasksNow(False)
         self.tasksCommand.setTimeToStartTasks(parsedMsg["schedule"]["start_time"])
         self.tasksCommand.setTimeToEndTasks(parsedMsg["schedule"]["end_time"])
      else:
         print(f"Received robot command: {self.robot_command}")
         self.tasksCommand.setStartTasksNow(False)
         self.tasksCommand.setTimeToStartTasks("00:00:00")
         self.tasksCommand.setTimeToEndTasks("00:00:00")
   
   def resetMsg(self):
      self.msg = []
      self.tasksCommand.setStartTasksNow(False)
      self.tasksCommand.setTimeToStartTasks("00:00:00")

   def isCtrlCommand(self) -> bool:
      return self.robot_command != "start" and self.robot_command != "schedule"
