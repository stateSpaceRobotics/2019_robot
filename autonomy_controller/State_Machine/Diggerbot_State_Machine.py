#!/usr/bin/env python
# The state machine of the Diggerbot

import roslib
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
from move_base_msgs.msg import *

# define state Idle
#


class Idle(smach.State):
    def __init__(self):
        self.dumper_in_pos = False
        sub = rospy.Subscriber(
            '/smach_flags/dumper_in_pos', Bool, self.callback)
        smach.State.__init__(self,
                             outcomes=['outcome1', 'outcome2', 'kill'],
                             input_keys=['Idle_counter_in',
                                         'e_stop'],
                             output_keys=['Idle_counter_out'])

    def execute(self, userdata):
        # rospy.loginfo('Executing state Idle')
        if userdata.e_stop == True:
            return 'kill'
        if self.dumper_in_pos == True:
            rospy.loginfo('[DIGGER]: dumper_in_pos was True')
            return 'outcome1'
        else:
            rospy.sleep(2)
            return 'outcome2'

    def callback(self, data):
        self.dumper_in_pos = data.data

# define state Dig_Prep
#


class Dig_Prep(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['Success', 'kill'], input_keys=['e_stop'])

    def execute(self, userdata):
        rospy.sleep(60)
        if userdata.e_stop == True:
            return 'kill'
        # rospy.loginfo('Executing state Dig_Prep')
        return 'Success'

# define state Dig
#


class Dig(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['kill', 'Success'], input_keys=['e_stop'])

    def execute(self, userdata):
        if userdata.e_stop == True:
            return 'kill'
        # rospy.loginfo('Executing state Dig')
        return 'Success'

# define state Load_Prep


class Load_Prep(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['Minibot_in_place', 'kill'], input_keys=['e_stop'])

    def execute(self, userdata):
        rospy.sleep(60)
        if userdata.e_stop == True:
            return 'kill'
        # rospy.loginfo('Executing Load Prep')
        return 'Minibot_in_place'

# define state Load
#


class Load(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['Load_Successful', 'kill'], input_keys=['e_stop'])

    def execute(self, userdata):
        if userdata.e_stop == True:
            return 'kill'
        # rospy.loginfo('Executing Load')
        return 'Load_Successful'

# define state Stuck
#


class Stuck(smach.State):
    def __init__(self):
        smach.State.__init__(
            self, outcomes=['Stuck', 'kill'], input_keys=['e_stop'])

    def execute(self, userdata):
        if userdata.e_stop == True:
            return 'kill'
        # rospy.loginfo('Praying for Mercy')
        return 'Stuck'

# define state Drive
# Zach is responsible for path planning


class Drive(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1', 'kill', 'stuck'],
                             input_keys=['Drive_counter_in', 'e_stop'])

    def execute(self, userdata):
        if userdata.e_stop == True:
            return 'kill'
        # rospy.loginfo('Executing state Drive')
        # rospy.loginfo('Counter = %f' % userdata.Drive_counter_in)
        return 'outcome1'

# define state Kill
#


class Kill(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Kill'])

    def execute(self, userdata):
        # rospy.loginfo('Executing Kill')
        return 'Kill'


def diggerbot_main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm_diggerbot = smach.StateMachine(outcomes=['end'])
    sm_diggerbot.userdata.sm_counter = 0
    sm_diggerbot.userdata.e_stop = False

    # Open the container
    with sm_diggerbot:
        # Add states to the container

        # STATE IDLE

        smach.StateMachine.add('Idle', Idle(),
                               transitions={'outcome1': 'Drive',
                                            'kill': 'Kill',
                                            'outcome2': 'Idle'},
                               remapping={'Idle_counter_in': 'sm_counter',
                                          'e_stop': 'e_stop',
                                          'Idle_counter_out': 'sm_counter'})

# STATE DRIVE

#        smach.StateMachine.add('Drive', Drive(), transitions={'outcome1': 'CON',
#                                                              'kill': 'Kill',
#                                                              'stuck': 'Stuck'},
#                               remapping={'Drive_counter_in': 'sm_counter',
#                                          'e_stop': 'e_stop'})

        def drive_goal_cb(userdata, goal):
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/map"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.position.x = 2.0
            drive_goal.target_pose.pose.position.y = 6.0
            drive_goal.target_pose.pose.orientation.w = 1.0

            return drive_goal

        def drive_result_cb(userdata, status, result):
            rospy.loginfo('[DIGGER]: status is %s', status)
            pub = rospy.Publisher(
                '/smach_flags/digger_in_pos', Bool, queue_size=1, latch=True)
            pub.publish(True)

        smach.StateMachine.add('Drive',
                               smach_ros.SimpleActionState('/digger/move_base',
                                                           MoveBaseAction,
                                                           goal_cb=drive_goal_cb,
                                                           result_cb=drive_result_cb,
                                                           input_keys=[
                                                               'goal_pose']),
                               transitions={'succeeded': 'CON',
                                            'aborted': 'Stuck',
                                            'preempted': 'Drive'},
                               remapping={'goal_pose': 'userdata_goal_pose'})

# STATE MACHINE LOADING
        sm_loader = smach.StateMachine(outcomes=['end loading'])
        sm_loader.userdata.e_stop = sm_diggerbot.userdata.e_stop
        sm_loader.userdata.sm_counter = sm_diggerbot.userdata.sm_counter

# STATE MACHINE DIGGINER
        sm_digger = smach.StateMachine(outcomes=['end digging'])
        sm_digger.userdata.e_stop = sm_diggerbot.userdata.e_stop
        sm_digger.userdata.sm_counter = sm_diggerbot.userdata.sm_counter

# STATES IN LOADING
        with sm_loader:

            # STATE LOAD PREP
            smach.StateMachine.add('Load_Prep', Load_Prep(),
                                   transitions={'kill': 'end loading',
                                                'Minibot_in_place': 'Load'},
                                   remapping={'e_stop': 'e_stop'})

# STATE LOAD
            smach.StateMachine.add('Load', Load(),
                                   transitions={'kill': 'end loading',
                                                'Load_Successful': 'Load_Prep'},
                                   remapping={'e_stop': 'e_stop'})

# STATE IN DIGGING
        with sm_digger:

            # STATE DIG PREP
            smach.StateMachine.add('Dig_Prep', Dig_Prep(),
                                   transitions={'kill': 'end digging',
                                                'Success': 'Dig'},
                                   remapping={'e_stop': 'e_stop'})

# STATE DIG
            smach.StateMachine.add('Dig', Dig(),
                                   transitions={'kill': 'end digging',
                                                'Success': 'Dig'},
                                   remapping={'e_stop': 'e_stop'})

# STATE MACHINE CONCURRENCE
        sm_con = smach.Concurrence(outcomes=['loop', 'end'],
                                   default_outcome='loop',
                                   outcome_map={'end':
                                                {'sm_digger': 'end digging',
                                                 'sm_loader': 'end loading'}})

# STATES IN CONCURRENCE
        with sm_con:

            # STATE SM_DIGGER
            smach.Concurrence.add('sm_digger', sm_digger)

# STATE SM_LOADER
            smach.Concurrence.add('sm_loader', sm_loader)

# STATE CONCURRENCE
        smach.StateMachine.add('CON', sm_con,
                               transitions={'end': 'Kill',
                                            'loop': 'CON'},
                               remapping={'e_stop': 'e_stop'})

# STATE STUCK

        smach.StateMachine.add('Stuck', Stuck(), transitions={'Stuck': 'Idle',
                                                              'kill': 'Kill'},
                               remapping={'e_stop': 'e_stop'})

# STATE KILL

        smach.StateMachine.add('Kill', Kill(), transitions={'Kill': 'end'})

    return sm_diggerbot

    # Execute SMACH plan
    # Execute SMACH plan
    # First you create a state machine sm
    # .....
    # Creating of state machine sm finished
    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('server_name', sm_diggerbot, '/SM_ROOT')
    # sis.start()

    # Execute the state machine
    # outcome = sm_diggerbot.execute()
    # Wait for ctrl-c to stop the application
    # rospy.spin()
    # sis.stop()


if __name__ == '__main__':
    diggerbot_main()
