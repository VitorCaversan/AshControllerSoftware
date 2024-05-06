import schedule
import time
import queue
from signal import signal, SIGTERM, SIGHUP, pause
from bluetoothTask import BluetoothTask
from taskScheduler import TaskScheduler
from auxClasses.tasksCommand import TasksCommand

msgQueue = queue.Queue()
bluetoothTask = BluetoothTask()
taskScheduler = TaskScheduler(queue=msgQueue)

def safeExit(self, signum, frame):
   bluetoothTask.stop()
   taskScheduler.stopRoutine()
   msgQueue.join()
   exit(1)

def main():
   bluetoothTask.start()

   while True:
      try:
         signal(SIGTERM, safeExit)
         signal(SIGHUP, safeExit)

         schedule.run_pending()

         tasks = bluetoothTask.getTasksCommand()

         if tasks.mustStartNow():
            taskScheduler.runRoutineNow()
            bluetoothTask.rxBtMsg.resetMsg()
         elif tasks.haveTimeToStart():
            timeToStart = tasks.getTimeToStartTasks()
            taskScheduler.runRoutineAt(timeToStart[0], timeToStart[1], timeToStart[2])

         print(str(msgQueue.qsize()))
         msg = msgQueue.get(timeout=1)

         if msg == "Ball stuck":
            print("Ball stuck")
            # Send message to Bluetooth device

         time.sleep(1)
      
      except KeyboardInterrupt:
         bluetoothTask.stop()
         taskScheduler.stopRoutine()
         msgQueue.join()
         exit(1)
      
      except queue.Empty:
         pass


main()