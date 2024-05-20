import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def pose_callback(data):
    # Esta função será chamada sempre que uma nova mensagem de pose for recebida
    print(data)

def main():
    rospy.init_node('robot_info_listener', anonymous=True)
    rospy.Subscriber("/base_scan", LaserScan, pose_callback)
    # Substitua '/robot_pose' pelo tópico que fornece a pose do robô no seu simulador
    rospy.spin()

if __name__ == '__main__':
    main()
