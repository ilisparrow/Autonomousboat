import numpy as np
import rospy
from geometry_msgs.msg import Point
import time

class ObstacleAvoid():
    def __init__(self):
        self.sub = rospy.Subscriber("/pos_obst", Point, self.callback)
        #suscriber topic etat commande mavros (Manual, Auto):
        self.substate = rospy.Subscriber("/mavros/state", State , self.state)
        #publier à commande vitesse mavros :
        self.pub = rospy.Publisher("", Twist, queue_size=10)
        rospy.init_node('avoid_obst', anonymous=True)
        self.rate = rospy.Rate(2)
        self.msg = Twist()
        self.scan = Point()
        self.state = False

    def callback(self, data):
        self.scan = data

    def state(self, data):
        self.state = (data.mode == 'Auto')

    def cmd_vit(self, point):
        #point.x = distance dist_min
        #point.y = angle deg
        #return angulaire z (+ gauche, - droite), linear x
        if point.x > 1 : #zone 2
            if point.y < 60 : #zone D
                return 0.0, 5.0
            if point.y < 90 : #zone C
                return 5.0, 4.0
            elif point.y < 120 : #zone B
                return -5.0, 4.0
            else: #zone A
                return 0.0, 5.0
        elif point.x > 0.5 :#zone1
            if point.y < 60 : #zone D
                return 0.0, 5.0
            elif point.y < 90 : #zone C
                return 5.0, 3.0
            elif point.y < 120 : #zone B
                return -5.0, 3.0
            else: #zone A
                return 0.0, 5.0
        else : #zone0
            if point.y < 60 : #zone D
                return 0.0, 5.0
            elif point.y < 90 : #zone C
                return 5.0, 2.0
            elif point.y < 120 : #zone B
                return -5.0, 2.0
            else: #zone A
                return 0.0, 5.0

    def run(self):
        while not rospy.is_shutdown():
            if self.state:
                self.msg.angular.x = 0
                self.msg.angular.y = 0
                self.msg.linear.y = 0
                self.msg.linear.z = 0
                self.msg.angulaire.z, self.msg.linear.x = cmd_vit(self.scan)
                self.pub.publish(self.msg)
                print("linear : {}, angular : {}".format(self.msg.angulaire.z, self.msg.linear.x))

test = ObstacleAvoid()
test.run()
