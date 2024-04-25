import schedule
import time
from bluetoothTask import BluetoothTask
from taskScheduler import TaskScheduler
from auxClasses.tasksCommand import TasksCommand

bluetoothTask = BluetoothTask()
taskScheduler = TaskScheduler()

def main():
   bluetoothTask.start()

   while True:
      try:
         schedule.run_pending()

         tasks = bluetoothTask.getTasksCommand()

         if tasks.mustStartNow():
            taskScheduler.runRoutineNow()
            bluetoothTask.rxBtMsg.resetMsg()
         elif tasks.haveTimeToStart():
            timeToStart = tasks.getTimeToStartTasks()
            taskScheduler.runRoutineAt(timeToStart[0], timeToStart[1], timeToStart[2])

         time.sleep(1)
      
      except KeyboardInterrupt:
         bluetoothTask.stop()
         taskScheduler.stopRoutine()
         exit(1)


main()