import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist   
from sensor_msgs.msg import LaserScan

import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.searchingBase import searchingBase

class ReturnBaseSim():
    def __init__(self) -> None:
        # Initialize nodes
        rospy.init_node('Ash_robot', anonymous=True)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
    
        rospy.Subscriber("/base_scan_0", LaserScan, self.laser_0_callback)

        rospy.Subscriber("/base_scan_1", LaserScan, self.laser_1_callback)

        rospy.Subscriber("/base_scan_2", LaserScan, self.laser_2_callback)

        rospy.Subscriber("/base_scan_3", LaserScan, self.laser_3_callback)

        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10) 

        get_laser = [self.get_last_laser_0, self.get_last_laser_1, self.get_last_laser_2, self.get_last_laser_3]

        self.nav = searchingBase(self.get_last_odom, get_laser, self.set_speed)

        self.latest_odom = None

    def odom_callback(self, data):
        self.latest_odom = [data.pose.pose.position.x, data.pose.pose.position.y]

    def get_last_odom(self):
        return self.latest_odom

    def laser_0_callback(self, data):
        self.latest_laser0 = data.ranges

    def get_last_laser_0(self):
        return self.latest_laser0

    def laser_1_callback(self, data):
        self.latest_laser1 = data.ranges

    def get_last_laser_1(self):
        return self.latest_laser1

    def laser_2_callback(self, data):
        self.latest_laser2 = data.ranges

    def get_last_laser_2(self):
        return self.latest_laser2

    def laser_3_callback(self, data):
        self.latest_laser3 = data.ranges

    def get_last_laser_3(self):
        return self.latest_laser3

    def set_speed(self, linear, angular):
        move_cmd = Twist()
        move_cmd.linear.x = linear
        move_cmd.angular.z = angular
        self.cmd_vel.publish(move_cmd) 

    def run(self):
        # Frequência de 10 Hz
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        # TODO: Handle odom and laser in callback to input in the nav.run() function
        self.nav.run()
        # rate.sleep()

if __name__ == '__main__':
    sim = ReturnBaseSim()
    sim.run()
    print('end')
