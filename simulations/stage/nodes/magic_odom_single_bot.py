#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, PoseArray, Pose

class PFieldNavigator(object):

    def __init__(self):
        '''
        Potential field navigator constructor.
        '''
        rospy.init_node("magic_stage_odom_robot_publisher")
        self.pub_dict = {}
        self.pub_dict2 = {}
        self.last_poses = {}
        self.miniBot_poses0 = PoseArray()
        self.miniBot_poses0.poses.append(Pose())

        ######################################
        # Setup ROS publishers
        #   but nothing automatically publishes
        ######################################
        # self.drive_pub = rospy.Publisher("miniBot_poses", PoseArray, queue_size = 10)
        self.miniBot_poses0_pub = rospy.Publisher("/robot_0/miniBot_poses", PoseArray, queue_size = 10)

        ######################################
        # Setup ROS subscibers
        ######################################
        # rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_modelstates_callback)
        rospy.Subscriber("/robot_0/odom", Odometry, self.stage_robot0_callback)

    def stage_robot0_callback(self, data):
        '''
        Callback for robot_0/odom
        '''
        newPose = data.pose.pose #since odom has pose with covariance you need to get out first
        self.miniBot_poses0_pub.publish(self.miniBot_poses0)

    def run(self):
        '''
        Do nothing, everything done in callbacks
        '''
        rospy.spin()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == "__main__":
    navigator = PFieldNavigator()
    navigator.run()
