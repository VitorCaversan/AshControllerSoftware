import threading
import time
from enum import Enum
from auxClasses.rxBluetoothMsg import RxBluetoothMsg

class BluetoothState(Enum):
   IDLE = 1
   SCANNING = 2
   CONNECTING = 3
   CONNECTED = 4
   DISCONNECTING = 5
   DISCONNECTED = 6

class BluetoothTask:
   def __init__(self):
      self.name = "BluetoothTask"
      self.description = "BluetoothTask"
      self.status = BluetoothState.IDLE
      self.dependencies = []
      self.rxBtMsg = RxBluetoothMsg("0_16_00:00:00".encode('utf-8'))
      self.thread = threading.Thread(target=self.listen)

   def listen(self):
      while(1):
         print("Listening for Bluetooth devices...")
         self.status = BluetoothState.SCANNING
         time.sleep(5)

   def start(self):
      self.thread.start()
   def stop(self):
      self.thread.join()

   def stopListening(self):
      self.thread.join()
      print("Bluetooth task stopped")

   def getTasksCommand(self):
      return self.rxBtMsg.tasksCommand

