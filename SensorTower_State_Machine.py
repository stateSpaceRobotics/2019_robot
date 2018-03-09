#!/usr/bin/env python
#The state machine of the Diggerbot

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define state Idle
# 
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['Idle_counter_in'],
                             output_keys=['Idle_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
        if userdata.Idle_counter_in < 3:
            userdata.Idle_counter_out = userdata.Idle_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'

class Dump_Prep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_in_place'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Dump Prep')
        return 'Minibot_in_place'

# define state Dig
#
class Dump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_empty'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Dump')
        return 'Minibot_empty'

# define state Load
#
class Kill(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Kill'])
    def execute(self, userdata):
        rospy.loginfo('Executing Kill')
        return 'Kill'

# define state Stuck
# 
class Stuck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Stuck'])
    def execute(self, userdata):
        rospy.loginfo('Praying for Mercy')
        return 'Stuck'

# define state Drive
#Lucas is responsible for path planning
class Drive(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2'],
                             input_keys=['Drive_counter_in'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Drive')
        rospy.loginfo('Counter = %f'%userdata.Drive_counter_in)        
        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Idle', Idle(), 
                               transitions={'outcome1':'Drive', 
                                            'outcome2':'outcome4'},
                               remapping={'Idle_counter_in':'sm_counter', 
                                          'Idle_counter_out':'sm_counter'})
        smach.StateMachine.add('Drive',Drive(),transitions={'outcome1':'Dump_Prep','outcome2':'Stuck'},
                                               remapping={'Drive_counter_in':'sm_counter'})
        smach.StateMachine.add('Dump_Prep',Dump_Prep(),transitions={'Minibot_in_place':'Dump'})
        smach.StateMachine.add('Dump', Dump(),transitions={'Minibot_empty':'Dump_Prep'})
        smach.StateMachine.add('Kill', Kill(), transitions={'Kill':'outcome4'})
        smach.StateMachine.add('Stuck',Stuck(), transitions={'Stuck':'Idle'})


    # Execute SMACH plan
    # Execute SMACH plan
    # First you create a state machine sm
    # .....
    # Creating of state machine sm finished
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
