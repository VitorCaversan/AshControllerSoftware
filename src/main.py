import schedule
import time
import queue
from signal import signal, SIGTERM, SIGHUP, pause
from bluetoothTask import BluetoothTask
from taskScheduler import TaskScheduler
from auxClasses.tasksCommand import TasksCommand

mainMsgQueue   = queue.Queue()
ctrlMsgQueue   = queue.Queue()
bluetoothTask  = BluetoothTask(ctrlMsgQueue)
taskScheduler  = TaskScheduler(mainQueue=mainMsgQueue, ctrlQueue=ctrlMsgQueue)

def safeExit(self, signum, frame):
   bluetoothTask.stop()
   taskScheduler.stopRoutine()
   mainMsgQueue.join()
   exit(1)

def main():
   bluetoothTask.start()
   taskScheduler.runRoutineNow()
   while True:
      try:
         signal(SIGTERM, safeExit)
         signal(SIGHUP, safeExit)

         schedule.run_pending()

         tasks = bluetoothTask.getTasksCommand()
         
         if tasks.mustStartNow():
            
            bluetoothTask.rxBtMsg.resetMsg()
         elif tasks.haveTimeToStart():
            timeToStart = tasks.getTimeToStartTasks()
            taskScheduler.runRoutineAt(timeToStart[0], timeToStart[1], timeToStart[2])

         msg = mainMsgQueue.get(timeout=1)

         if msg != "":
            bluetoothTask.sendRobotStatus(msg)
            mainMsgQueue.task_done()

         time.sleep(1)
      
      except KeyboardInterrupt:
         bluetoothTask.stop()
         taskScheduler.stopRoutine()
         mainMsgQueue.join()
         exit(1)
      
      except queue.Empty:
         pass


main()