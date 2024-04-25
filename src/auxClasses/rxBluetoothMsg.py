from enum import Enum
from auxClasses.tasksCommand import TasksCommand

class MsgType(Enum):
   START_NOW = 0
   SCHEDULE_START = 1
   IDLE = 2

# A class containing all info needed to handle task scheduling
class RxBluetoothMsg:
   def __init__(self, msg):
      self.msg = msg.decode('utf-8').split('_')
      self.msg_type = MsgType(int(self.msg[0]))
      self.msg_len = self.msg[1]
      self.msg_data = self.msg[2:]

      self.tasksCommand = TasksCommand()
      self.parseMsg(msg)

   def parseMsg(self, msg):
      self.msg = msg.decode('utf-8').split('_')
      self.msg_type = MsgType(int(self.msg[0]))
      self.msg_len = self.msg[1]
      self.msg_data = self.msg[2]

      if self.msg_type == MsgType.START_NOW:
         self.tasksCommand.setStartTasksNow(True)
      elif self.msg_type == MsgType.SCHEDULE_START:
         self.tasksCommand.setStartTasksNow(False)
         self.tasksCommand.setTimeToStartTasks(self.msg_data)
      else:
         print("Unknown message type")
         self.tasksCommand.setStartTasksNow(False)
         self.tasksCommand.setTimeToStartTasks("00:00:00")
   
   def resetMsg(self):
      self.msg = []
      self.msg_type = MsgType.IDLE
      self.msg_len = 0
      self.msg_data = []
      self.tasksCommand.setStartTasksNow(False)
      self.tasksCommand.setTimeToStartTasks("00:00:00")