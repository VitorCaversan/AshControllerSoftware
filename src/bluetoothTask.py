import threading
import time
import socket
import queue
import subprocess
import os
import json
import bluetooth as bt # This is the PyBluez library. 
# To install:
# sudo apt-get install bluetooth libbluetooth-dev
# pip install git+https://github.com/pybluez/pybluez.git#egg=pybluez (--break-system-packages if pip complains about something)
# If it still doesn't recognize the bluetooth import:
# sudo apt install python3-bluez
# To solve bluetooth.btcommon.BluetoothError: (2, 'No such file or directory'):
# Run the Bluetooth daemon in 'compatibility' mode. Edit /etc/systemd/system/dbus-org.bluez.service and add '-C' after 'bluetoothd'. Reboot.
# Then sudo sdptool add SP
from auxClasses.rxBluetoothMsg import RxBluetoothMsg

# To successfully run this bluetooth server, these following commands
# must be done in the raspberry pi terminal to make it publically visible:
# bluetoothctl
# power on
# discoverable on
# pairable on
# agent NoInputNoOutput
# default-agent
# exit

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
      self.server: socket = None
      self.uuid = "7be1fcb3-5776-42fb-91fd-2ee7b5bbb86d"
      self.client: socket = None
      self.ctrlMsgQueue = ctrlMsgQueue
      self.thread = threading.Thread(target=self.listen)

   def listen(self):
      '''
      Serves a socket on the default port, listening for clients.  Upon client connection, runs a loop to 
      that receives period-delimited messages from the client and calls the sub-class's 
      handleMessage(self, message) method.   Sub-class can call send(self, message) to send a 
      message back to the client.   Begins listening again after client disconnects.
      '''

      # Make device visible
      os.system("sudo hciconfig hci0 piscan")
      # maybe also sudo sdptool add SP

      # Create a new server socket using RFCOMM protocol
      self.server = bt.BluetoothSocket(bt.RFCOMM)

      # Bind to any port
      self.server.bind(("", bt.PORT_ANY))

      # Start listening
      self.server.listen(1)

      # Get the port the server socket is listening
      port = self.server.getsockname()[1]

      # Start advertising the service
      bt.advertise_service(self.server, "RaspiBtSrv",
                        service_id=self.uuid,
                        service_classes=[self.uuid, bt.SERIAL_PORT_CLASS],
                        profiles=[bt.SERIAL_PORT_PROFILE])
      while True:
         print(f"Waiting for connection on RFCOMM channel {port}")

         try:
            self.client, address = self.server.accept()
            print(f"Connected with {address}")

            while True:
               data = self.client.recv(1024).decode('utf-8')
               if data:
                  self.rxBtMsg.parseMsg(data)
                  okMsg = {"status" : statusCodes[200],
                           "message": "Robot started"}
                  self.client.send((json.dumps(okMsg) + '\n').encode('utf-8')) # App decodes messages up until '\n'

                  print(f"Received message: {self.rxBtMsg.msg}")
                  
                  if self.rxBtMsg.isCtrlCommand():
                     self.ctrlMsgQueue.put(self.rxBtMsg.robot_command)
               
               time.sleep(5)
         except socket.error as e:
            print(f"Socket error: {e}")
         except IOError:
            pass
         except KeyboardInterrupt:
            if self.client is not None:
               self.client.close()

            self.server.close()

            print("Server going down")
            break

   def makeDiscoverable(self):
      subprocess.run("bluetoothctl power on", shell=True)
      subprocess.run("bluetoothctl agent on", shell=True)
      subprocess.run("bluetoothctl default-agent", shell=True)
      subprocess.run("bluetoothctl agent NoInputNoOutput", shell=True)
      subprocess.run("bluetoothctl discoverable on", shell=True)
      subprocess.run("bluetoothctl pairable on", shell=True)

   def sendRobotStatus(self, json: str):
      if self.client:
         print(f"Sending message: {json}")
         self.client.send((json + '\n').encode('utf-8')) # App decodes messages up until '\n'

   def start(self):
      self.thread.start()
   def stop(self):
      self.thread.join()
      self.server.close()
      if self.client is not None:
         self.client.close()

   def stopListening(self):
      self.thread.join()
      print("Bluetooth task stopped")

   def getTasksCommand(self):
      return self.rxBtMsg.tasksCommand

