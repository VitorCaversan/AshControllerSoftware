import schedule
import time
import queue
from signal import signal, SIGTERM, SIGHUP, pause
from bluetoothTask import BluetoothTask
from taskScheduler import TaskScheduler
from auxClasses.tasksCommand import TasksCommand

queue = queue.Queue()
bluetoothTask = BluetoothTask()
taskScheduler = TaskScheduler(queue=queue)

def safeExit(self, signum, frame):
   bluetoothTask.stop()
   taskScheduler.stopRoutine()
   queue.join()
   exit(1)

def main():
   bluetoothTask.start()

   while True:
      try:
         msg = queue.get_nowait()

         if msg == "Ball stuck":
            print("Ball stuck")
            # Send message to Bluetooth device
      except queue.Empty:
         pass
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

         time.sleep(1)
         pause()
      
      except KeyboardInterrupt:
         bluetoothTask.stop()
         taskScheduler.stopRoutine()
         queue.join()
         exit(1)


main()