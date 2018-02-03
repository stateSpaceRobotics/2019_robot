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
        self.miniBot_poses1 = PoseArray()
        self.miniBot_poses2 = PoseArray()
        self.miniBot_poses3 = PoseArray()

        self.miniBot_poses0.poses.append(Pose())
        self.miniBot_poses1.poses.append(Pose())
        self.miniBot_poses2.poses.append(Pose())
        self.miniBot_poses3.poses.append(Pose())

        self.miniBot_poses0.poses.append(Pose())
        self.miniBot_poses1.poses.append(Pose())
        self.miniBot_poses2.poses.append(Pose())
        self.miniBot_poses3.poses.append(Pose())

        self.miniBot_poses0.poses.append(Pose())
        self.miniBot_poses1.poses.append(Pose())
        self.miniBot_poses2.poses.append(Pose())
        self.miniBot_poses3.poses.append(Pose())
        
        ######################################
        # Setup ROS publishers
        #   but nothing automatically publishes
        ######################################
        # self.drive_pub = rospy.Publisher("miniBot_poses", PoseArray, queue_size = 10)
        self.miniBot_poses0_pub = rospy.Publisher("/robot_0/miniBot_poses", PoseArray, queue_size = 10)
        self.miniBot_poses1_pub = rospy.Publisher("/robot_1/miniBot_poses", PoseArray, queue_size = 10)
        self.miniBot_poses2_pub = rospy.Publisher("/robot_2/miniBot_poses", PoseArray, queue_size = 10)
        self.miniBot_poses3_pub = rospy.Publisher("/robot_3/miniBot_poses", PoseArray, queue_size = 10)

        ######################################
        # Setup ROS subscibers
        ######################################
        # rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_modelstates_callback)
        rospy.Subscriber("/robot_0/odom", Odometry, self.stage_robot0_callback)
        rospy.Subscriber("/robot_1/odom", Odometry, self.stage_robot1_callback)
        rospy.Subscriber("/robot_2/odom", Odometry, self.stage_robot2_callback)
        rospy.Subscriber("/robot_3/odom", Odometry, self.stage_robot3_callback)

    def stage_robot0_callback(self, data):
        '''
        Callback for robot_0/odom
        '''
        newPose = data.pose.pose #since odom has pose with covariance you need to get out first
        self.miniBot_poses1.poses[0] = newPose
        self.miniBot_poses2.poses[0] = newPose
        self.miniBot_poses3.poses[0] = newPose
        self.miniBot_poses0_pub.publish(self.miniBot_poses0)

    def stage_robot1_callback(self, data):
        '''
        Callback for robot_1/odom
        '''
        newPose = data.pose.pose #since odom has pose with covariance you need to get out first
        self.miniBot_poses0.poses[0] = newPose
        self.miniBot_poses2.poses[1] = newPose
        self.miniBot_poses3.poses[1] = newPose
        self.miniBot_poses1_pub.publish(self.miniBot_poses1)

    def stage_robot2_callback(self, data):
        '''
        Callback for robot_2/odom
        '''
        newPose = data.pose.pose #since odom has pose with covariance you need to get out first
        self.miniBot_poses1.poses[1] = newPose
        self.miniBot_poses0.poses[1] = newPose
        self.miniBot_poses3.poses[2] = newPose
        self.miniBot_poses2_pub.publish(self.miniBot_poses2)

    def stage_robot3_callback(self, data):
        '''
        Callback for robot_3/odom
        '''
        newPose = data.pose.pose #since odom has pose with covariance you need to get out first
        self.miniBot_poses1.poses[2] = newPose
        self.miniBot_poses2.poses[2] = newPose
        self.miniBot_poses0.poses[2] = newPose
        self.miniBot_poses3_pub.publish(self.miniBot_poses3)

    def gazebo_modelstates_callback(self, data):
        '''
        Callback for model states
        '''
        i = 0
        poses = PoseArray()
        while i < len(data.name):
            if (data.name[i]).startswith(MODELPREFIX):
                name = data.name[i]
                pose = data.pose[i]
                twist = data.twist[i]
                odom = Odometry()
                odom.pose.pose = pose
                odom.twist.twist = twist

                self.last_poses[name] = pose
                poses.poses = self.last_poses.values()
                poses.poses.remove(pose)
                if name not in self.pub_dict:
                    self.pub_dict[name] = rospy.Publisher("/"+name+"/odom", Odometry, queue_size = 10)
                    self.pub_dict2[name] = rospy.Publisher("/"+name+"/miniBot_poses", PoseArray, queue_size = 10)

                self.pub_dict[name].publish(odom)
                self.pub_dict2[name].publish(poses)
            i += 1
        # self.drive_pub.publish(poses)
        

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
