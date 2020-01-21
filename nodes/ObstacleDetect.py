import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import time

class ObstacleDetect():

    def __init__(self):
        self.sub = rospy.Subscriber("/scan",LaserScan, self.callback)
        #self.pub = rospy.Publisher("/ObstacleAvoid", Point, queue_size=10)
        rospy.init_node('ObstacleDetect', anonymous=True)
        self.rate = rospy.Rate(2)
        self.msg = Point()
        self.scan = LaserScan()

    def callback(self, data):
        self.scan = data

    def RAD2DEG(x):
        return (x*180.)/np.pi

    def run(self):
        while not rospy.is_shutdown():
            if not (len(self.scan.ranges)==0):
                rospy.loginfo('ObstacleDetect boucle :')
                dist_min = min(self.scan.ranges)
                print("angle_min {1}, angle_max {2}, angle_increment {3}, scan len {4}".format(self.scan.angle_min, self.scan.angle_max, self.scan.angle_increment, len(self.scan.ranges)))
                if dist_min < 2.5:
                    #distance :
                    self.msg.x = min(scan.ranges)
                    #angle :
                    self.msg.y = RAD2DEG(self.scan.angle_min + scan.ranges.index(min(scan.ranges[]))*self.scan.angle_increment)
                    self.msg.z = 0
                    #self.pub.publish(self.msg)
                    print("L'obstacle est a {1}m et {2}deg".format(self.msg.x, self.msg.y))

test = ObstacleDetect()
test.run()

#pour rendre le code executable :
#chmod u+x ~/catkin_ws/src/turtlesim_cleaner/src/move.py
