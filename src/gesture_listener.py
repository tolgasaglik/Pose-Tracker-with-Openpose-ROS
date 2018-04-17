#!/usr/bin/env python

import rospy
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


def hand_raise(human):
    return (((human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0)
             or (human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0))
            and not ((human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0)
                     and (human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0)))


def wait_a_minute(human):
    return ((human.body_key_points_with_prob[C_RIGHT_SHOULDER].y > human.body_key_points_with_prob[C_RIGHT_WRIST].y > 0)
            and (human.body_key_points_with_prob[C_LEFT_WRIST].x > human.body_key_points_with_prob[C_NECK].x > human.body_key_points_with_prob[C_RIGHT_WRIST].x)
            and (human.body_key_points_with_prob[C_LEFT_SHOULDER].y > human.body_key_points_with_prob[C_LEFT_WRIST].y > 0))


def break_activity(human):
    return ((human.body_key_points_with_prob[C_LEFT_WRIST].x < human.body_key_points_with_prob[C_NECK].x < human.body_key_points_with_prob[C_RIGHT_WRIST].x)
            and (human.body_key_points_with_prob[C_LEFT_WRIST].y < human.body_key_points_with_prob[C_RIGHT_SHOULDER].y)
            and (human.body_key_points_with_prob[C_RIGHT_WRIST].y < human.body_key_points_with_prob[C_LEFT_SHOULDER].y))


def callback(msg):
    for human in msg.human_list:  # type: object
        ind = msg.human_list.index(human)  # type: object

        rospy.loginfo("Gesture from Human: " + str(ind))
        # for key_point in human.body_key_points_with_prob[2:8]:
        #     x = key_point.x
        #     y = key_point.y
        #     prob = key_point.prob
        #     rospy.loginfo('x: {}, y: {}, prob: {}'.format(x, y, prob))
        if hand_raise(human):
            rospy.loginfo("Human:" + ind + " is raising a HAND!")
            self.publisher_msg.gesture_list.append((ind, str(G_HAND_RAISE)))
            self.pub.publish(self.publisher_msg)
        elif wait_a_minute(human):
            rospy.loginfo("Human:" + ind + " tells PEPPER to wait!")
        elif break_activity(human):
            rospy.loginfo("Human:" + ind + " tells PEPPER to break activity!")
        rospy.loginfo("\n")


def gesture_detector():
    rospy.init_node("gesture_detector", anonymous=True)
    self.pub = rospy.Publisher("/gesture_detector/gesture_list", GestureDetectorHumanList, queue_size=10)
    rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, callback)
    r = rospy.Rate(1)

    self.publisher_msg = GestureDetectorHumanList()
    gesture_list = GestureDetectorHuman()
    while not rospy.is_shutdown():
        # Publish visible human index
        # publisher_msg.num_humans = str(OpenPoseHumanList.human_list)
        # for human in OpenPoseHumanList.human_list:
        #     if hand_raise(human):
        #         publisher_msg.gesture_list.append((ind, G_HAND_RAISE))
        #     elif wait_a_minute(human):
        #         publisher_msg.gesture_list.append((ind, G_WAIT_A_MINUTE))
        #     elif break_activity(human):
        #         publisher_msg.gesture_list.append((ind, G_BREAK_ACTIVITY))
        # pub.publish(publisher_msg)
        r.sleep()  # sleep for one second
    # rospy.spin()


if __name__ == '__main__':
    try:
        gesture_detector()
    except rospy.ROSInterruptException:
        pass
