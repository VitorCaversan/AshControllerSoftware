import threading
import time
import socket
import queue
from enum import Enum
from auxClasses.rxBluetoothMsg import RxBluetoothMsg

statusCodes = {
   200 : "200 OK",
   400 : "400 Bad Request",
   404 : "404 Not Found",
   409 : "409 Conflict",
   503 : "503 Service Unavailable"
}

class BluetoothTask:
   def __init__(self, ctrlMsgQueue: queue.Queue):
      self.name = "BluetoothTask"
      self.description = "BluetoothTask"
      self.rxBtMsg = RxBluetoothMsg()
      self.server = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
      self.server.bind(("B8:27:EB:8E:C4:59", 4))
      self.server.listen(1)
      self.client: socket = None
      self.ctrlMsgQueue = ctrlMsgQueue
      self.thread = threading.Thread(target=self.listen)

   def listen(self):
      try:
         client, address = self.server.accept()
         self.client = client
         print(f"Connected with {address}")

         while(1):
            data = self.client.recv(1024).decode('utf-8')
            if data:
               self.rxBtMsg.parseMsg(data)
               self.client.send(statusCodes[200].encode('utf-8'))

               print(f"Received message: {self.rxBtMsg.msg}")
               
               if self.rxBtMsg.isCtrlCommand():
                  self.ctrlMsgQueue.put(self.rxBtMsg.robot_command)
            else:
               print("No data received, closing connection")
               break
            time.sleep(5)
      except socket.error as e:
         print(f"Socket error: {e}")
      finally:
         self.server.close()
         print("Socket closed")

   def sendRobotStatus(self, json: str):
      if self.client:
         print(f"Sending message: {json}")
         self.client.send(json.encode('utf-8'))

   def start(self):
      self.thread.start()
   def stop(self):
      self.thread.join()
      self.client.close()

   def stopListening(self):
      self.thread.join()
      print("Bluetooth task stopped")

   def getTasksCommand(self):
      return self.rxBtMsg.tasksCommand

