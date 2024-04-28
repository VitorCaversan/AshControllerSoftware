import schedule
import time
from signal import signal, SIGTERM, SIGHUP, pause
from bluetoothTask import BluetoothTask
from taskScheduler import TaskScheduler
from auxClasses.tasksCommand import TasksCommand

bluetoothTask = BluetoothTask()
taskScheduler = TaskScheduler()

def safeExit(self, signum, frame):
   bluetoothTask.stop()
   taskScheduler.stopRoutine()
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

         time.sleep(1)
         pause()
      
      except KeyboardInterrupt:
         bluetoothTask.stop()
         taskScheduler.stopRoutine()
         exit(1)


main()