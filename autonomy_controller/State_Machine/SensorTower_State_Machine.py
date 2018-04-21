#!/usr/bin/env python
# The state machine of the Diggerbot

import roslib
import rospy
import smach
import smach_ros
import tf2_ros
from std_msgs.msg import Bool
from move_base_msgs.msg import *
from geometry_msgs.msg import Twist

# define state Idle
#


class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'kill'],
                             input_keys=['Idle_counter_in', 'e_stop'],
                             output_keys=['Idle_counter_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Idle')
        if userdata.e_stop == True:
            return 'kill'
        if userdata.Idle_counter_in < 3:
            userdata.Idle_counter_out = userdata.Idle_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'


class Dump_Prep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_in_place', 'kill'],
                             input_keys=['e_stop'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Dump Prep')
        rospy.sleep(60)
        if userdata.e_stop == True:
            return 'kill'
        return 'Minibot_in_place'

# define state Dig
#


class Dump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_empty', 'kill'],
                             input_keys=['e_stop'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Dump')
        if userdata.e_stop == True:
            return 'kill'
        return 'Minibot_empty'

# define state Load
#


class Kill(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Kill'])

    def execute(self, userdata):
        #rospy.loginfo('Executing Kill')
        return 'Kill'

# define state Stuck
#


class Stuck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Stuck', 'kill'],
                             input_keys=['e_stop'])

    def execute(self, userdata):
        #rospy.loginfo('Praying for Mercy')
        if userdata.e_stop == True:
            return 'kill'
        return 'Stuck'

# define state Drive
# Lucas is responsible for path planning


class Back_Up(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['repeat', 'finished', 'kill'],
                             input_keys=['e_stop'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Drive')
        if userdata.e_stop == True:
            return 'kill'

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        trans = tfBuffer.lookup_transform(
            'robot_0/base_link', 'vive', rospy.Time(), rospy.Duration(2.0))

        pub = rospy.Publisher(
            '/dumper/cmd_vel', Twist)
        cmd = Twist()

        if trans.transform.translation.x < -0.2:
            cmd.linear.x = -0.1
            pub.publish(cmd)
            return 'repeat'
        else:
            cmd.linear.x = 0
            pub.publish(cmd)
            return 'finished'


def sensortower_main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm_sensortower = smach.StateMachine(outcomes=['end'])
    sm_sensortower.userdata.sm_counter = 0
    sm_sensortower.userdata.e_stop = False
    sm_sensortower.userdata.rotation_array = [
        [0.960, -0.280], [0.843, 0.537], [0.960, -0.280]]
    global rotation_index
    rotation_index = 0

    # Open the container
    with sm_sensortower:
        # Add states to the container
        smach.StateMachine.add('Idle', Idle(),
                               transitions={'outcome1': 'Drive',
                                            'outcome2': 'end',
                                            'kill': 'Kill'},
                               remapping={'Idle_counter_in': 'sm_counter',
                                          'Idle_counter_out': 'sm_counter',
                                          'e_stop': 'e_stop'})
#        smach.StateMachine.add('Drive', Drive(),
#                               transitions={'outcome1': 'Dump_Prep',
#                                            'outcome2': 'Stuck',
#                                            'kill': 'Kill'},
#                               remapping={'Drive_counter_in': 'sm_counter',
#                                          'e_stop': 'e_stop'})

        def drive_goal_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/map"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.position.x = 1.89
            drive_goal.target_pose.pose.position.y = 0.5
            drive_goal.target_pose.pose.orientation.w = 0.707
            drive_goal.target_pose.pose.orientation.z = 0.707

            return drive_goal

        smach.StateMachine.add('Drive',
                               smach_ros.SimpleActionState('/dumper/move_base',
                                                           MoveBaseAction,
                                                           goal_cb=drive_goal_cb,
                                                           input_keys=[
                                                               'aruco_pose'
                                                           ]),
                               transitions={'succeeded': 'Rotation',
                                            'aborted': 'Stuck',
                                            'preempted': 'Drive'},
                               remapping={'aruco_pose': 'userdata_aruco_pose'})

        def rotation_goal_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/robot_0/base_link"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.orientation.w = userdata.rotation_array[
                rotation_index][0]
            drive_goal.target_pose.pose.orientation.z = userdata.rotation_array[
                rotation_index][1]

            return drive_goal

        @smach.cb_interface(outcomes=['finished'])
        def rotation_result_cb(userdata, status, result):
            global rotation_index
            rotation_index += 1
            if (rotation_index == 3):
                rospy.loginfo('[SENSOR]: status is %s', status)
                pub = rospy.Publisher(
                    '/smach_flags/dumper_in_pos', Bool, queue_size=1, latch=True)
                pub.publish(True)
                return 'finished'

        smach.StateMachine.add('Rotation',
                               smach_ros.SimpleActionState('/dumper/move_base',
                                                           MoveBaseAction,
                                                           goal_cb=rotation_goal_cb,
                                                           result_cb=rotation_result_cb,
                                                           input_keys=[
                                                               'rotation_array'
                                                           ]),
                               transitions={'succeeded': 'Rotation',
                                            'aborted': 'Stuck',
                                            'preempted': 'Drive',
                                            'finished': 'Back_Up'},
                               remapping={'rotation_array': 'rotation_array'})

        smach.StateMachine.add('Back_Up', Back_Up(),
                               transitions={'finished': 'Dump_Prep',
                                            'repeat': 'Back_Up',
                                            'kill': 'Kill'},
                               remapping={'e_stop': 'e_stop'})

        smach.StateMachine.add('Dump_Prep', Dump_Prep(),
                               transitions={'Minibot_in_place': 'Dump',
                                            'kill': 'Kill'},
                               remapping={'e_stop': 'e_stop'})
        smach.StateMachine.add('Dump', Dump(),
                               transitions={'Minibot_empty': 'Dump_Prep',
                                            'kill': 'Kill'},
                               remapping={'e_stop': 'e_stop'})
        smach.StateMachine.add('Kill', Kill(),
                               transitions={'Kill': 'end'})
        smach.StateMachine.add('Stuck', Stuck(),
                               transitions={'Stuck': 'Idle',
                                            'kill': 'Kill'},
                               remapping={'e_stop': 'e_stop'})

    return sm_sensortower

    # Execute SMACH plan
    # Execute SMACH plan
    # First you create a state machine sm
    # .....
    # Creating of state machine sm finished
    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()


    # Execute the state machine
    #outcome = sm_sensortower.execute()
    # Wait for ctrl-c to stop the application
    # rospy.spin()
    # sis.stop()
if __name__ == '__main__':
    sensortower_main()
