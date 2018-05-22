#!/usr/bin/env python

import rospy
import time, threading, datetime
from std_msgs.msg import Int32, String
from openpose_ros_msgs.msg import OpenPoseHumanList
from gesture_detector.msg import *

# BODY CONSTANTS
C_NOSE = 0
C_NECK = 1
C_RIGHT_SHOULDER = 2
C_RIGHT_ELBOW = 3
C_RIGHT_WRIST = 4
C_LEFT_SHOULDER = 5
C_LEFT_ELBOW = 6
C_LEFT_WRIST = 7
C_RIGHT_HIP = 8
C_RIGHT_KNEE = 9
C_RIGHT_ANKLE = 10
C_LEFT_HIP = 11
C_LEFT_KNEE = 12
C_LEFT_ANKLE = 13
C_RIGHT_EYE = 14
C_LEFT_EYE = 15
C_RIGHT_EAR = 16
C_LEFT_EAR = 17
C_BACKGROUND = 18

# FACE for QT
F_ANGRY = "angry"
F_BLINK = "blink"
F_DISGUST = "disgust"
F_FEAR = "fear"
F_HAPPY = "ava_happy"
F_LOGO = "logo"
F_SAD = "ava_sad"
F_SUPRISE = "suprise"


class Pose_Tracker(object):
    """Gesture Detection Node on ROS using Openpose"""
    def __init__(self):
        # Publisher for user interaction durations
        #self.pub = rospy.Publisher("/pose_tracker/interaction_details", interaction_details, queue_size=10)
        # Publisher for QT robot
        self.pub2 = rospy.Publisher("/face_duration/setEmotion", String, queue_size=1)
        #self.publisher_msg = PoseTrackerHumanList()

        """Initialize ROS components"""
        self.idle_time_start = time.time()
        self.contact_time_start = time.time()
        self.elapsed_idle_time = 0.
        self.elapsed_contact_time = 0.
        self.previous_timestamp = time.time()
        self.init_node()


    """Method: Initialize ROS components"""
    def init_node(self):
        self.pose_tracker()


    """Pose Trackers"""
    def isFaceDistracted(self, human):
        return (
            (human.body_key_points_with_prob[C_RIGHT_EAR].x == 0 and human.body_key_points_with_prob[C_RIGHT_EAR].y == 0) 
            or (human.body_key_points_with_prob[C_LEFT_EAR].x == 0 and human.body_key_points_with_prob[C_LEFT_EAR].y == 0)
            or (human.body_key_points_with_prob[C_NECK].y - human.body_key_points_with_prob[C_NOSE].y < 100) 
            or (human.body_key_points_with_prob[C_NECK].y - human.body_key_points_with_prob[C_NOSE].y > 150)
            or (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].x < human.body_key_points_with_prob[C_RIGHT_SHOULDER].x)
        )
    

    """Operational Functions """
    def callback(self, msg):
        #self.publisher_msg.num_humans = len(msg.human_list)
        i = 0
        face_gesture = ""
        human_count = len(msg.human_list)

        for human in msg.human_list:
            # Check gesture published by current human and publish respective message
            if self.isFaceDistracted(human):
                rospy.loginfo("Human:" + str(i) + " is NOT looking! :-(")
                face_gesture = F_SAD
                self.elapsed_idle_time += time.time() - self.previous_timestamp
            else:
                rospy.loginfo("Human:" + str(i) + " is looking :-)")
                face_gesture = F_HAPPY
                self.elapsed_contact_time += time.time() - self.previous_timestamp

            i += 1
            rospy.loginfo("\n")
            rospy.loginfo("Contact time in seconds: %s" %(self.elapsed_contact_time))
            rospy.loginfo("--")
            rospy.loginfo("Idle time in seconds: %s" %(self.elapsed_idle_time))
            rospy.loginfo("--")
            rospy.loginfo("Total time in seconds: %s" %(self.elapsed_idle_time + self.elapsed_contact_time))
            rospy.loginfo("--")
            performance = self.elapsed_contact_time/(self.elapsed_contact_time+self.elapsed_idle_time)*100
            rospy.loginfo("Focus performance: %s" %performance)
            rospy.loginfo("\n")

            self.previous_timestamp = time.time()

        # self.pub.publish(interaction_details)
        self.pub2.publish(face_gesture)


    def pose_tracker(self):
        rospy.init_node("pose_tracker", anonymous=True)
        rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, self.callback)
        #pub_period = rospy.get_param("~pub_period",5.0)
        #rospy.Timer(rospy.Duration.from_sec(pub_period),self.callback)

        r = rospy.Rate(0.01)
        while not rospy.is_shutdown():
            #rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, self.callback)
            r.sleep()  # sleep for 1 second
        # rospy.spin()


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting ....")
        node = Pose_Tracker()
        node.init_node()
    except rospy.ROSInterruptException:
        pass
