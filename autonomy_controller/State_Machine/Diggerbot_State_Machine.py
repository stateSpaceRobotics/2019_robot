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

#define state Dig_Prep
#
class Dig_Prep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Success','kill'],input_keys=['e_stop'])
    def execute(self, userdata):
        if userdata.e_stop==True:
            return 'kill'
        rospy.loginfo('Executing state Dig_Prep')
        return 'Success'

# define state Dig
#
class Dig(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Full_Bucket','kill'],input_keys=['e_stop'])
    def execute(self, userdata):
        if userdata.e_stop==True:
            return 'kill'
        rospy.loginfo('Executing state Dig')
        return 'Full_Bucket'

# define state Load_Prep
class Load_Prep(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_in_place','kill'],input_keys=['e_stop'])
    def execute(self, userdata):
        if userdata.e_stop==True:
            return 'kill'
        rospy.loginfo('Executing Load Prep')
        return 'Minibot_in_place'

# define state Load
#
class Load(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Load_Successful','kill'],input_keys=['e_stop'])
    def execute(self, userdata):
        if userdata.e_stop==True:
            return 'kill'
        rospy.loginfo('Executing Load')
        return 'Load_Successful'

# define state Stuck
# 
class Stuck(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Stuck','kill'],input_keys=['e_stop'])
    def execute(self, userdata):
        if userdata.e_stop==True:
            return 'kill'
        rospy.loginfo('Praying for Mercy')
        return 'Stuck'

# define state Drive
#Zach is responsible for path planning
class Drive(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','kill','stuck'],
                             input_keys=['Drive_counter_in','e_stop'])
        
    def execute(self, userdata):
        if userdata.e_stop==True:
            return 'kill'
        rospy.loginfo('Executing state Drive')
        rospy.loginfo('Counter = %f'%userdata.Drive_counter_in)        
        return 'outcome1'

# define state Kill
#
class Kill(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Kill'])
    def execute(self, userdata):
        rospy.loginfo('Executing Kill')
        return 'Kill'




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0
    sm.userdata.e_stop = False

    # Open the container
    with sm:
        # Add states to the container

#STATE IDLE

        smach.StateMachine.add('Idle', Idle(), 
                               transitions={'outcome1':'Drive', 
                                            'kill':'Kill',
                                            'outcome2':'outcome4'},
                               remapping={'Idle_counter_in':'sm_counter',
                                          'e_stop':'e_stop', 
                                          'Idle_counter_out':'sm_counter'})

#STATE DRIVE

        smach.StateMachine.add('Drive',Drive(),transitions={'outcome1':'Dig_Prep',
                                                            'kill':'Kill',
                                                            'stuck':'Stuck'},
                                               remapping={'Drive_counter_in':'sm_counter',
                                                          'e_stop':'e_stop'})

#STATE DIG_Prep

        smach.StateMachine.add('Dig_Prep',Dig_Prep(),transitions={'Success':'Dig',
                                                                  'kill':'Kill'},
                                                     remapping={'e_stop':'e_stop'})

#STATE DIG

        smach.StateMachine.add('Dig', Dig(),transitions={'Full_Bucket':'Load_Prep',
                                                         'kill':'Kill'},
                                            remapping={'e_stop':'e_stop'})

#STATE LOAD_PREP

        smach.StateMachine.add('Load_Prep', Load_Prep(), transitions={'Minibot_in_place':'Load',
                                                                      'kill':'Kill'},
                                                         remapping={'e_stop':'e_stop'})

#STATE LOAD

        smach.StateMachine.add('Load', Load(), transitions={'Load_Successful':'Dig_Prep',
                                                            'kill':'Kill'},
                                               remapping={'e_stop':'e_stop'})

#STATE STUCK

        smach.StateMachine.add('Stuck',Stuck(), transitions={'Stuck':'Idle',
                                                             'kill':'Kill'},
                                                remapping={'e_stop':'e_stop'})

#STATE KILL

        smach.StateMachine.add('Kill',Kill(), transitions={'Kill':'outcome4'})


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
