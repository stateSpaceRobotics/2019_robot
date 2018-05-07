#!/usr/bin/env python
# The state machine of the Minibot


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
        self.digger_in_pos = False
        sub = rospy.Subscriber(
            '/smach_flags/digger_in_pos', Bool, self.callback)
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'kill'],
                             input_keys=['Idle_counter_in', 'e_stop'],
                             output_keys=['Idle_counter_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Idle')
        if userdata.e_stop == True:
            return 'kill'
        if self.digger_in_pos == True:
            rospy.loginfo('[Minibot]: Digger_in_pos was True')
            return 'outcome1'
        else:
            rospy.sleep(2)
            return 'outcome2'

    def callback(self, data):
        self.digger_in_pos = data.data

# define state Wait_to_Load


class Wait_to_load(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Full_Bucket', 'kill'], input_keys=[
                             'counter_in', 'e_stop'], output_keys=['counter_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Wait_to_load')
        if userdata.e_stop == True:
            return 'kill'
        return 'Full_Bucket'

# define state Dig
#


class Wait_to_dump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Empty_Bucket', 'Sensors Lost', 'kill'], input_keys=[
                             'counter_in', 'e_stop'], output_keys=['counter_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Wait_to_dump')
        if userdata.e_stop == True:
            return 'kill'
        return 'Empty_Bucket'

# define state Load_Prep


class Lost(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_lost', 'kill'], input_keys=[
                             'counter_in', 'e_stop'], output_keys=['counter_out'])

    def execute(self, userdata):
        #rospy.loginfo('Executing Lost')
        if userdata.e_stop == True:
            return 'kill'
        return 'Minibot_lost'

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
        smach.State.__init__(self, outcomes=['Stuck', 'kill'], input_keys=[
                             'counter_in', 'e_stop'], output_keys=['counter_out'])

    def execute(self, userdata):
        #rospy.loginfo('Praying for Mercy')
        if userdata.e_stop == True:
            return 'kill'
        return 'Stuck'

# define state Drive
# Lucas is responsible for path planning


class Back_Up_Load(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished', 'repeat', 'kill'],
                             input_keys=['e_stop'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Drive')
        #rospy.loginfo('Counter = %f' % userdata.Drive_counter_in)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        trans = tfBuffer.lookup_transform(
            'robot_2/base_link', 'robot_1/base_link', rospy.Time(), rospy.Duration(2.0))

        pub = rospy.Publisher(
            '/transporter/cmd_vel', Twist)
        cmd = Twist()

        if trans.transform.translation.x < 0.7:
            cmd.linear.x = -0.1
            pub.publish(cmd)
            return 'repeat'
        else:
            cmd.linear.x = 0.0
            pub.publish(cmd)
            return 'finished'


class Back_Up_Dump(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['finished', 'repeat', 'kill'],
                             input_keys=['e_stop'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Drive')
        #rospy.loginfo('Counter = %f' % userdata.Drive_counter_in)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        trans = tfBuffer.lookup_transform(
            'robot_2/base_link', 'robot_0/base_link', rospy.Time(), rospy.Duration(2.0))

        pub = rospy.Publisher(
            '/transporter/cmd_vel', Twist)
        cmd = Twist()

        if trans.transform.translation.x < 0.7:
            cmd.linear.x = -0.1
            pub.publish(cmd)
            return 'repeat'
        else:
            cmd.linear.x = 0.0
            pub.publish(cmd)
            return 'finished'


def minibot_main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm_minibot = smach.StateMachine(outcomes=['end'])
    sm_minibot.userdata.sm_counter = 0
    sm_minibot.userdata.e_stop = False

    # Open the container
    with sm_minibot:
        # Add states to the container

        # STATE IDLE

        smach.StateMachine.add('Idle', Idle(),
                               transitions={'outcome1': 'Drive_to_digger',
                                            'outcome2': 'Idle',
                                            'kill': 'Kill'},
                               remapping={'Idle_counter_in': 'sm_counter',
                                          'e_stop': 'e_stop',
                                          'Idle_counter_out': 'sm_counter'})

# STATE DRIVE

#        smach.StateMachine.add('Drive',Drive(),transitions={'outcome1':'Wait_to_load',
#                                                            'outcome2':'Wait_to_dump',
#                                                            'outcome3':'Stuck',
#                                                            'outcome4':'Lost',
#                                                            'kill':'Kill'},
#                                                remapping={'Drive_counter_in':'sm_counter',
#                                                           'e_stop':'e_stop'})

        def drive_to_digger_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/robot_1/base_link"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.position.x = -1.0
            drive_goal.target_pose.pose.orientation.w = 1.0

            return drive_goal

        smach.StateMachine.add('Drive_to_digger',
                               smach_ros.SimpleActionState('/transporter/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb=drive_to_digger_cb,
                                                           input_keys=[
                                                               'goal_pose'
                                                           ]),
                               transitions={'succeeded': 'Load_Prep',
                                            'aborted': 'Stuck',
                                            'preempted': 'Drive_to_digger'},
                               remapping={'goal_pose': 'userdata_goal_post'})

        def load_prep_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/robot_1/base_link"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.position.x = -0.55
            drive_goal.target_pose.pose.orientation.w = 1.0

            return drive_goal

        smach.StateMachine.add('Load_Prep',
                               smach_ros.SimpleActionState('/transporter/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb=load_prep_cb,
                                                           input_keys=[
                                                               'goal_pose'
                                                           ]),
                               transitions={'succeeded': 'Wait_to_load',
                                            'aborted': 'Stuck',
                                            'preempted': 'Load_Prep'},
                               remapping={'goal_pose': 'userdata_goal_post'})

# STATE WAIT_TO_LOAD

        smach.StateMachine.add('Wait_to_load', Wait_to_load(), transitions={'Full_Bucket': 'Back_Up_Load',
                                                                            'kill': 'Kill'},
                               remapping={'counter_in': 'sm_counter',
                                          'e_stop': 'e_stop',
                                          'counter_out': 'sm_counter'})

# STATE BACK_UP_LOAD

        smach.StateMachine.add('Back_Up_Load', Back_Up_Load(), transitions={'finished': 'Turn_from_digger',
                                                                            'repeat': 'Back_Up_Load',
                                                                            'kill': 'Kill'},
                               remapping={'counter_in': 'sm_counter',
                                          'e_stop': 'e_stop',
                                          'counter_out': 'sm_counter'})

        def turn_from_digger_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/robot_2/base_link"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.orientation.w = 0.707
            drive_goal.target_pose.pose.orientation.z = 0.707

            return drive_goal

        smach.StateMachine.add('Turn_from_digger',
                               smach_ros.SimpleActionState('/transporter/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb=turn_from_digger_cb,
                                                           input_keys=[
                                                               'goal_pose'
                                                           ]),
                               transitions={'succeeded': 'Drive_to_dumper',
                                            'aborted': 'Stuck',
                                            'preempted': 'Turn_from_digger'},
                               remapping={'goal_pose': 'userdata_goal_post'})

        def drive_to_dumper_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/robot_0/base_link"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.position.x = 0.7
            drive_goal.target_pose.pose.orientation.z = 1

            return drive_goal

        smach.StateMachine.add('Drive_to_dumper',
                               smach_ros.SimpleActionState('/transporter/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb=drive_to_dumper_cb,
                                                           input_keys=[
                                                               'goal_pose'
                                                           ]),
                               transitions={'succeeded': 'Dump_Prep',
                                            'aborted': 'Stuck',
                                            'preempted': 'Drive_to_dumper'},
                               remapping={'goal_pose': 'userdata_goal_post'})

        def dump_prep_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/robot_0/base_link"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.position.x = 0.5
            drive_goal.target_pose.pose.orientation.z = 1.0

            return drive_goal

        smach.StateMachine.add('Dump_Prep',
                               smach_ros.SimpleActionState('/transporter/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb=dump_prep_cb,
                                                           input_keys=[
                                                               'goal_pose'
                                                           ]),
                               transitions={'succeeded': 'Wait_to_dump',
                                            'aborted': 'Stuck',
                                            'preempted': 'Dump_Prep'},
                               remapping={'goal_pose': 'userdata_goal_post'})
# STATE WAIT_TO_DUMP

        smach.StateMachine.add('Wait_to_dump', Wait_to_dump(), transitions={'Empty_Bucket': 'Back_Up_Dump',
                                                                            'Sensors Lost': 'Lost',
                                                                            'kill': 'Kill'},
                               remapping={'counter_in': 'sm_counter',
                                          'e_stop': 'e_stop',
                                          'counter_out': 'sm_counter'})

        smach.StateMachine.add('Back_Up_Dump', Back_Up_Dump(), transitions={'finished': 'Turn_from_dumper',
                                                                            'repeat': 'Back_Up_Dump',
                                                                            'kill': 'Kill'},
                               remapping={'counter_in': 'sm_counter',
                                          'e_stop': 'e_stop',
                                          'counter_out': 'sm_counter'})

        def turn_from_dumper_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/robot_2/base_link"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.orientation.z = 1.0

            return drive_goal

        smach.StateMachine.add('Turn_from_dumper',
                               smach_ros.SimpleActionState('/transporter/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb=turn_from_dumper_cb,
                                                           input_keys=[
                                                               'goal_pose'
                                                           ]),
                               transitions={'succeeded': 'Drive_to_digger',
                                            'aborted': 'Stuck',
                                            'preempted': 'Turn_from_dumper'},
                               remapping={'goal_pose': 'userdata_goal_post'})

# STATE LOST

        smach.StateMachine.add('Lost', Lost(), transitions={'Minibot_lost': 'Idle',
                                                            'kill': 'Kill'},
                               remapping={'counter_in': 'sm_counter',
                                          'e_stop': 'e_stop',
                                          'counter_out': 'sm_counter'})

# STATE KILL

        smach.StateMachine.add('Kill', Kill(), transitions={'Kill': 'end'})

# STATE STUCK

        smach.StateMachine.add('Stuck', Stuck(), transitions={'Stuck': 'Idle',
                                                              'kill': 'Kill'},
                               remapping={'counter_in': 'sm_counter',
                                          'e_stop': 'e_stop',
                                          'counter_out': 'sm_counter'})

    return sm_minibot

    # Execute SMACH plan
    # First you create a state machine sm
    # .....
    # Creating of state machine sm finished
    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    # Execute the state machine
    #outcome = sm_minibot.execute()
    # Wait for ctrl-c to stop the application
    # rospy.spin()
    # sis.stop()


if __name__ == '__main__':
    minibot_main()
