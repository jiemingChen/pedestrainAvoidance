#!/usr/bin/env python

import rospy
import time
import sys
from geometry_msgs.msg import Twist


class MovePerson(object):
    def __init__(self, person_model_name):
        person_moving_topic = "/" + person_model_name + "/cmd_vel"
        self._move_base_publisher = rospy.Publisher(person_moving_topic, Twist, queue_size=1)

    def move_to(self, twist_object):
        self._move_base_publisher.publish(twist_object)
    
    def stop(self):
        stop_twist = Twist()
        self.move_to(stop_twist)
    
    def oscilate_left_right(self, n=30):
        twist_object = Twist()

        self.stop()
        time.sleep(2.5)
        print("0000")

        twist_object.angular.z = -1
        self.move_to(twist_object)
        time.sleep(5.0)
        print("11111")
        
        twist_object.angular.z = 0
        twist_object.linear.x = -.8
        self.move_to(twist_object)
        time.sleep(5)
        print("1.51.51.51.51.5")
 
        twist_object.angular.z = 1
        twist_object.linear.x = 0
        self.move_to(twist_object)
        time.sleep(5.0)
        print("2222")
        
        twist_object.angular.z = 0
        twist_object.linear.x = .8
        self.move_to(twist_object)
        time.sleep(5)
        
        self.stop()



def OscilatePerson():
    # Create a node
    rospy.init_node("move_person_oscilate_node")

    move_person = MovePerson(model_name)
    s = "Y"
    while not rospy.is_shutdown() and s == "Y":
        move_person.oscilate_left_right()
        s="N"
    
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("usage: move_person.py model_name")
    else:
        model_name = sys.argv[1]
    OscilatePerson()

