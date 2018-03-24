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
                             outcomes=['outcome1','outcome2','kill'],
                             input_keys=['Idle_counter_in','e_stop'],
                             output_keys=['Idle_counter_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state Idle')
        if userdata.e_stop==True:
            return 'kill'
        if userdata.Idle_counter_in < 3:
            userdata.Idle_counter_out = userdata.Idle_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'

class Dump_Prep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_in_place','kill'],
                                   input_keys=['e_stop'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Dump Prep')
        if userdata.e_stop==True:
            return 'kill'
        return 'Minibot_in_place'

# define state Dig
#
class Dump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_empty','kill'],
                                   input_keys=['e_stop'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Dump')
        if userdata.e_stop==True:
            return 'kill'
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
        smach.State.__init__(self, outcomes=['Stuck','kill'],
                                   input_keys=['e_stop'])
    def execute(self, userdata):
        rospy.loginfo('Praying for Mercy')
        if userdata.e_stop==True:
            return 'kill'
        return 'Stuck'

# define state Drive
#Lucas is responsible for path planning
class Drive(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2','kill'],
                             input_keys=['Drive_counter_in','e_stop'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Drive')
        if userdata.e_stop==True:
            return 'kill'
        rospy.loginfo('Counter = %f'%userdata.Drive_counter_in)        
        return 'outcome1'
        




def sensortower_main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm_sensortower = smach.StateMachine(outcomes=['end'])
    sm_sensortower.userdata.sm_counter = 0
    sm_sensortower.userdata.e_stop=False

    # Open the container
    with sm_sensortower:
        # Add states to the container
        smach.StateMachine.add('Idle', Idle(), 
                               transitions={'outcome1':'Drive', 
                                            'outcome2':'end',
                                            'kill':'Kill'},
                               remapping={'Idle_counter_in':'sm_counter', 
                                          'Idle_counter_out':'sm_counter',
                                          'e_stop':'e_stop'})
        smach.StateMachine.add('Drive',Drive(),
                               transitions={'outcome1':'Dump_Prep',
                                            'outcome2':'Stuck',
                                            'kill':'Kill'},
                               remapping={'Drive_counter_in':'sm_counter',
                                          'e_stop':'e_stop'})
        smach.StateMachine.add('Dump_Prep',Dump_Prep(),
                               transitions={'Minibot_in_place':'Dump',
                                            'kill':'Kill'},
                               remapping={'e_stop':'e_stop'})
        smach.StateMachine.add('Dump', Dump(),
                               transitions={'Minibot_empty':'Dump_Prep',
                                            'kill':'Kill'},
                               remapping={'e_stop':'e_stop'})
        smach.StateMachine.add('Kill', Kill(), 
                               transitions={'Kill':'end'})
        smach.StateMachine.add('Stuck',Stuck(), 
                               transitions={'Stuck':'Idle',
                                            'kill':'Kill'},
                               remapping={'e_stop':'e_stop'})

    return sm_sensortower

    # Execute SMACH plan
    # Execute SMACH plan
    # First you create a state machine sm
    # .....
    # Creating of state machine sm finished
    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()

    # Execute the state machine
    #outcome = sm_sensortower.execute()
    # Wait for ctrl-c to stop the application
    #rospy.spin()
    #sis.stop()
if __name__ == '__main__':
    main()
