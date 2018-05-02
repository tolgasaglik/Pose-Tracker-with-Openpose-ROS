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
G_REACH_UP = 5
G_BEND_RIGHT = 6
G_BEND_LEFT = 7

# GESTURE CONSTANTS STRINGS
GS_HAND_RAISE = "HAND"
GS_WAIT_A_MINUTE = "STOP"
GS_BREAK_ACTIVITY = "BREAK"
GS_GO_RIGHT = "RIGHT"
GS_GO_LEFT = "LEFT"
GS_REACH_UP = "REACH_UP"
GS_BEND_RIGHT = "BEND_RIGHT"
GS_BEND_LEFT = "BEND_LEFT"


class Gesture_Detector(object):
    """Gesture Detection Node on ROS using Openpose"""
    def __init__(self):
        self.pub = rospy.Publisher("/gesture_detector/gesture_list", GestureDetectorHumanList, queue_size=10)
        self.publisher_msg = GestureDetectorHumanList()
        """Initialize ROS components"""
        self.init_node()

    """Method: Initialize ROS components"""

    def init_node(self):
        self.gesture_detector()

    """Gesture Recognizers"""
    def hand_raise(self, human):
        return ((((human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0)
                 or (human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0))
                and not ((human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0)
                         and (human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0)))
                and ((human.body_key_points_with_prob[C_RIGHT_SHOULDER].x > human.body_key_points_with_prob[C_RIGHT_WRIST].x > 0) 
                    or (human.body_key_points_with_prob[C_LEFT_WRIST].x > human.body_key_points_with_prob[C_LEFT_SHOULDER].x > 0)))

    def wait_a_minute(self, human):
        return ((human.body_key_points_with_prob[C_RIGHT_ELBOW].y > human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0)
                and (human.body_key_points_with_prob[C_LEFT_WRIST].x > human.body_key_points_with_prob[C_NECK].x > human.body_key_points_with_prob[C_RIGHT_WRIST].x)
                and (human.body_key_points_with_prob[C_LEFT_ELBOW].y > human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0)
                and (human.body_key_points_with_prob[C_LEFT_WRIST].x - human.body_key_points_with_prob[C_RIGHT_WRIST].x) > (human.body_key_points_with_prob[C_LEFT_SHOULDER].x - human.body_key_points_with_prob[C_RIGHT_SHOULDER].x))

    def break_activity(self, human):
        return ((human.body_key_points_with_prob[C_LEFT_SHOULDER].x > human.body_key_points_with_prob[C_LEFT_WRIST].x > human.body_key_points_with_prob[C_RIGHT_SHOULDER].x > 0)
                and (human.body_key_points_with_prob[C_LEFT_SHOULDER].x > human.body_key_points_with_prob[C_RIGHT_WRIST].x > human.body_key_points_with_prob[C_RIGHT_SHOULDER].x > 0)
                and (0 < human.body_key_points_with_prob[C_NOSE].y < human.body_key_points_with_prob[C_RIGHT_WRIST].y < human.body_key_points_with_prob[C_RIGHT_ELBOW].y)
                and (0 < human.body_key_points_with_prob[C_NOSE].y < human.body_key_points_with_prob[C_LEFT_WRIST].y < human.body_key_points_with_prob[C_LEFT_ELBOW].y))

    def go_right(self, human):
        return ((0 < human.body_key_points_with_prob[C_NECK].x < human.body_key_points_with_prob[C_RIGHT_WRIST].x)
                and (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].x < human.body_key_points_with_prob[C_LEFT_ELBOW].x < human.body_key_points_with_prob[C_LEFT_WRIST].x)
                and (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].y < human.body_key_points_with_prob[C_LEFT_WRIST].y)
                and (0 < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y < human.body_key_points_with_prob[C_RIGHT_WRIST].y))

    def go_left(self, human):
        return ((0 < human.body_key_points_with_prob[C_LEFT_WRIST].x < human.body_key_points_with_prob[C_NECK].x)
                and (0 < human.body_key_points_with_prob[C_RIGHT_WRIST].x < human.body_key_points_with_prob[C_RIGHT_ELBOW].x < human.body_key_points_with_prob[C_RIGHT_SHOULDER].x)
                and (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].y < human.body_key_points_with_prob[C_LEFT_WRIST].y)
                and (0 < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y < human.body_key_points_with_prob[C_RIGHT_WRIST].y))

    def reach_up(self, human):
        return (0 < human.body_key_points_with_prob[C_RIGHT_WRIST].y < human.body_key_points_with_prob[C_RIGHT_ELBOW].y < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y
                and 0 < human.body_key_points_with_prob[C_LEFT_WRIST].y < human.body_key_points_with_prob[C_LEFT_ELBOW].y < human.body_key_points_with_prob[C_LEFT_SHOULDER].y
                and (human.body_key_points_with_prob[C_LEFT_WRIST].x - human.body_key_points_with_prob[C_RIGHT_WRIST].x) < 1.3 * (human.body_key_points_with_prob[C_LEFT_SHOULDER].x - human.body_key_points_with_prob[C_RIGHT_SHOULDER].x))

    def lateral_left_bend(self, human):
        return (0 < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y < human.body_key_points_with_prob[C_NECK].y < human.body_key_points_with_prob[C_LEFT_SHOULDER].y
                and 0 < human.body_key_points_with_prob[C_RIGHT_HIP].x < human.body_key_points_with_prob[C_RIGHT_SHOULDER].x
                and 0 < human.body_key_points_with_prob[C_LEFT_HIP].x < human.body_key_points_with_prob[C_NECK].x)

    def lateral_right_bend(self, human):
        return (0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].y < human.body_key_points_with_prob[C_NECK].y < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y
                and 0 < human.body_key_points_with_prob[C_LEFT_SHOULDER].x < human.body_key_points_with_prob[C_LEFT_HIP].x
                and 0 < human.body_key_points_with_prob[C_NECK].x < human.body_key_points_with_prob[C_RIGHT_HIP].x)
    """Gesture recognizers end"""

    """Operational Functions """
    def callback(self, msg):
        self.publisher_msg.num_humans = len(msg.human_list)

        # self.publisher_msg.gesture_list = [GestureDetectorHuman()] * len(msg.human_list)
        # rospy.loginfo("Visible humans:" + str(human_list_length))
        i = 0
        detected_human_list = GestureDetectorHumanList()
        detected_human_list.num_humans = len(msg.human_list)

        for human in msg.human_list:

            detected_human = GestureDetectorHuman()
            detected_human.human_index = i
            rospy.loginfo("Gesture from Human: " + str(i))

            # Check gesture published by current human and publish respective message
            if self.hand_raise(human):
                # rospy.loginfo("Human:" + str(ind) + " is raising a HAND!")
                detected_human.gesture = str(GS_HAND_RAISE)

            elif self.wait_a_minute(human):
                # rospy.loginfo("Human:" + str(ind) + " tells PEPPER to wait!")
                detected_human.gesture = str(GS_WAIT_A_MINUTE)

            elif self.break_activity(human):
                # rospy.loginfo("Human:" + str(ind) + " tells PEPPER to break activity!")
                detected_human.gesture = str(GS_BREAK_ACTIVITY)

            elif self.go_right(human):
                # rospy.loginfo("Human:" + str(ind) + " tells PEPPER to go right!")
                detected_human.gesture = str(GS_GO_RIGHT)

            elif self.go_left(human):
                # rospy.loginfo("Human:" + str(ind) + " tells PEPPER to go left!")
                detected_human.gesture = str(GS_GO_LEFT)

            elif self.reach_up(human):
                # rospy.loginfo("Human:" + str(ind) + " is reaching reach up!")
                detected_human.gesture = str(GS_REACH_UP)

            elif self.lateral_right_bend(human):
                # rospy.loginfo("Human:" + str(ind) + " is bending right!")
                detected_human.gesture = str(GS_BEND_RIGHT)

            elif self.lateral_left_bend(human):
                # rospy.loginfo("Human:" + str(ind) + " is bending left!")
                detected_human.gesture = str(GS_BEND_RIGHT)

            detected_human_list.gesture_list.append(detected_human)
            i += 1
            rospy.loginfo("\n")
            rospy.loginfo("--" + detected_human.gesture)
            # rospy.loginfo(detected_human_list)
            rospy.loginfo("\n")
            # rospy.loginfo(self.publisher_msg.gesture_list)

        self.pub.publish(detected_human_list)

    def gesture_detector(self):
        rospy.init_node("gesture_detector", anonymous=True)
        rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, self.callback)
        r = rospy.Rate(2)

        while not rospy.is_shutdown():
            r.sleep()  # sleep for one second
        # rospy.spin()


if __name__ == '__main__':
    try:
        rospy.loginfo("Starting ....")
        node = Gesture_Detector()
        node.init_node()
    except rospy.ROSInterruptException:
        pass
