#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry

def callback1(msg):
    
    new_msg = msg
    # new_msg.header.frame_id = new_frame1
    new_msg.child_frame_id = new_child_frame2  

    pub1.publish(new_msg)
    
def callback2(msg):
    
    new_msg = msg
    new_msg.header.frame_id = new_frame2
    new_msg.child_frame_id = new_child_frame2
   
    pub2.publish(new_msg)

if __name__ == '__main__':
    
    rospy.init_node('topic_utility', anonymous=True)

    new_frame1 = "husky1/map"
    new_frame2 = "husky2/map"
    
    new_child_frame1 = "husky1/base_link"
    new_child_frame2 = "husky2/base_link"
  
    sub1 = rospy.Subscriber('/husky2/odometry/gps', Odometry, callback1) 
    sub2 = rospy.Subscriber('/husky2/husky2/gazebo/odom', Odometry, callback2) 

    # Crea il publisher per il nuovo topic
    pub1 = rospy.Publisher('/husky2/gps', Odometry, queue_size=10)
    pub2 = rospy.Publisher('/husky2/odom', Odometry, queue_size=10)

    rospy.spin()
