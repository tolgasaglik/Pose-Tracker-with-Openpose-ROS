#!/usr/bin/env python

import rospy
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


# GESTURE CONSTANTS
G_HAND_RAISE = 0
G_WAIT_A_MINUTE = 1
G_BREAK_ACTIVITY = 2
G_GO_RIGHT = 3
G_GO_LEFT = 4


class Gesture_Detector(object):
    """Gesture Detection Node on ROS using Openpose"""

    # publisher_msg = GestureDetectorHumanList
    # gesture_list = GestureDetectorHuman()

    def __init__(self):
        self.publisher_msg = GestureDetectorHumanList()
        """Initialize ROS components"""
        self.init_node()

    """Method: Initialize ROS components"""

    def init_node(self):
        self.gesture_detector()

    """Gesture Recognizers"""

    def hand_raise(self, human):
        return (((human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0)
                 or (human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0))
                and not ((human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0)
                         and (human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0)))

    def wait_a_minute(self, human):
        return ((human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0)
                and (human.body_key_points_with_prob[C_LEFT_WRIST].x > human.body_key_points_with_prob[C_NECK].x > human.body_key_points_with_prob[C_RIGHT_WRIST].x)
                and (human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0))

    def break_activity(self, human):
        return ((human.body_key_points_with_prob[C_LEFT_SHOULDER].x > human.body_key_points_with_prob[C_LEFT_WRIST].x > human.body_key_points_with_prob[C_RIGHT_SHOULDER].x > 0)
                and (human.body_key_points_with_prob[C_LEFT_SHOULDER].x > human.body_key_points_with_prob[C_RIGHT_WRIST].x > human.body_key_points_with_prob[C_RIGHT_SHOULDER].x > 0)
                and (0 < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y < human.body_key_points_with_prob[C_RIGHT_WRIST].y < human.body_key_points_with_prob[C_RIGHT_ELBOW].y)
                and (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].y < human.body_key_points_with_prob[C_LEFT_WRIST].y < human.body_key_points_with_prob[C_LEFT_ELBOW].y))

    def go_right(self, human):
        return ((0 < human.body_key_points_with_prob[C_NECK].x < human.body_key_points_with_prob[C_RIGHT_WRIST].x)
                and (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].x < human.body_key_points_with_prob[C_LEFT_ELBOW].x < human.body_key_points_with_prob[C_LEFT_WRIST].x)
                and (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].y < human.body_key_points_with_prob[C_LEFT_WRIST].y)
                and (0 < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y < human.body_key_points_with_prob[C_RIGHT_WRIST].y))

    def go_leftt(self, human):
        return ((0 < human.body_key_points_with_prob[C_LEFT_WRIST].x < human.body_key_points_with_prob[C_NECK].x)
                and (0 < human.body_key_points_with_prob[C_RIGHT_WRIST].x < human.body_key_points_with_prob[C_RIGHT_ELBOW].x < human.body_key_points_with_prob[C_RIGHT_SHOULDER].x)
                and (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].y < human.body_key_points_with_prob[C_LEFT_WRIST].y)
                and (0 < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y < human.body_key_points_with_prob[C_RIGHT_WRIST].y))

    """Operational Functions"""

    def callback(self, msg):
        self.publisher_msg.num_humans = len(msg.human_list)
        for human in msg.human_list:  # type: object
            ind = msg.human_list.index(human)  # type: object
            rospy.loginfo("Gesture from Human: " + str(ind))
            self.publisher_msg.num_humans = len(msg.human_list)
            self.publisher_msg.gesture_list = [GestureDetectorHuman()] * (ind + 1)
            self.publisher_msg.gesture_list[msg.human_list.index(human)].human_index = msg.human_list.index(human)

            # Check gesture published by current human and publish respective message
            if self.hand_raise(human):
                self.publisher_msg.gesture_list[msg.human_list.index(human)].gesture = str(G_HAND_RAISE)
                rospy.loginfo("Human:" + str(ind) + " is raising a HAND!")
            elif self.wait_a_minute(human):
                self.publisher_msg.gesture_list[msg.human_list.index(human)].gesture = str(G_WAIT_A_MINUTE)
                rospy.loginfo("Human:" + str(ind) + " tells PEPPER to wait!")
            elif self.break_activity(human):
                self.publisher_msg.gesture_list[msg.human_list.index(human)].gesture = str(G_BREAK_ACTIVITY)
                rospy.loginfo("Human:" + str(ind) + " tells PEPPER to break activity!")
            elif self.go_right(human):
                self.publisher_msg.gesture_list[msg.human_list.index(human)].gesture = str(G_GO_RIGHT)
                rospy.loginfo("Human:" + str(ind) + " tells PEPPER to go right!")
            elif self.go_left(human):
                self.publisher_msg.gesture_list[msg.human_list.index(human)].gesture = str(G_GO_LEFT)
                rospy.loginfo("Human:" + str(ind) + " tells PEPPER to go left!")
            rospy.loginfo("\n")
            #self.publisher_msg.gesture_list[0].gesture = "home"
            # self.publisher_msg.gesture_list[1]= (ind, str(G_HAND_RAISE))

            # for key_point in human.body_key_points_with_prob[2:8]:
            #     x = key_point.x
            #     y = key_point.y
            #     prob = key_point.prob
            #     rospy.loginfo('x: {}, y: {}, prob: {}'.format(x, y, prob))
            # if self.hand_raise(human):
            #     rospy.loginfo("Human:" + str(ind) + " is raising a HAND!")
            #     self.publisher_msg.gesture_list.append((ind, str(G_HAND_RAISE)))
            #     self.pub.publish(self.publisher_msg)
            # elif self.wait_a_minute(human):
            #     rospy.loginfo("Human:" + str(ind) + " tells PEPPER to wait!")
            # elif self.break_activity(human):
            #     rospy.loginfo("Human:" + str(ind) + " tells PEPPER to break activity!")
            # rospy.loginfo("\n")

    def gesture_detector(self):
        rospy.init_node("gesture_detector", anonymous=True)
        self.pub = rospy.Publisher("/gesture_detector/gesture_list", GestureDetectorHumanList, queue_size=10)
        rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, self.callback)
        r = rospy.Rate(1)

        # publisher_msg = GestureDetectorHumanList()
        # gesture_list = GestureDetectorHuman()
        while not rospy.is_shutdown():
            # publish should stay in this loop to terminate the algorithm when CTRL-C is pressed
            # o/w terminal should be manually shut down
            self.pub.publish(self.publisher_msg)
            r.sleep()  # sleep for one second
        # rospy.spin()


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting ....")
        node = Gesture_Detector()
        node.init_node()
    except rospy.ROSInterruptException:
        pass
