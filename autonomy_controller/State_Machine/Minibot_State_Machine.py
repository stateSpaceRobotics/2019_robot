#!/usr/bin/env python
#The state machine of the Minibot



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
        if userdata.Idle_counter_in < 5:
            userdata.Idle_counter_out = userdata.Idle_counter_in + 1
            return 'outcome1'
        else:
            return 'outcome2'

#define state Wait_to_Load

class Wait_to_load(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Full_Bucket','kill'],input_keys=['counter_in','e_stop'],output_keys=['counter_out'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Wait_to_load')
        if userdata.e_stop==True:
            return 'kill'
        userdata.counter_out=userdata.counter_in+1
        return 'Full_Bucket'

# define state Dig
#
class Wait_to_dump(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Empty_Bucket','Sensors Lost','kill'],input_keys=['counter_in','e_stop'],output_keys=['counter_out'])
    def execute(self, userdata):
        rospy.loginfo('Executing state Wait_to_dump')
        if userdata.e_stop==True:
            return 'kill'
        userdata.counter_out=userdata.counter_in+1
        return 'Empty_Bucket'

# define state Load_Prep
class Lost(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Minibot_lost','kill'],input_keys=['counter_in','e_stop'],output_keys=['counter_out'])
    def execute(self, userdata):
        rospy.loginfo('Executing Lost')
        if userdata.e_stop==True:
            return 'kill'
        userdata.counter_out=userdata.counter_in+1
        return 'Minibot_lost'

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
        smach.State.__init__(self, outcomes=['Stuck','kill'],input_keys=['counter_in','e_stop'],output_keys=['counter_out'])
    def execute(self, userdata):
        rospy.loginfo('Praying for Mercy')
        if userdata.e_stop==True:
            return 'kill'
        userdata.counter_out=userdata.counter_in+1
        return 'Stuck'

# define state Drive
#Lucas is responsible for path planning
class Drive(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['outcome1','outcome2', 'outcome3', 'outcome4','kill'],
                             input_keys=['Drive_counter_in','e_stop'])
        
    def execute(self, userdata):
        rospy.loginfo('Executing state Drive')
        rospy.loginfo('Counter = %f'%userdata.Drive_counter_in)
        if userdata.e_stop==True:
            return 'kill'
        if userdata.Drive_counter_in==1:
            return 'outcome1'
        elif userdata.Drive_counter_in==2:
            return 'outcome2'
        elif userdata.Drive_counter_in==3:
            return 'outcome3'
        elif userdata.Drive_counter_in==4:
            return 'outcome4'
        elif userdata.Drive_counter_in==5:
            return 'outcome4'

        




def minibot_main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm_minibot = smach.StateMachine(outcomes=['end'])
    sm_minibot.userdata.sm_counter = 0
    sm_minibot.userdata.e_stop = False

    # Open the container
    with sm_minibot:
        # Add states to the container

#STATE IDLE

        smach.StateMachine.add('Idle', Idle(), 
                               transitions={'outcome1':'Drive', 
                                            'outcome2':'end',
                                            'kill':'Kill'},
                               remapping={'Idle_counter_in':'sm_counter',
                                          'e_stop':'e_stop', 
                                          'Idle_counter_out':'sm_counter'})

#STATE DRIVE

        smach.StateMachine.add('Drive',Drive(),transitions={'outcome1':'Wait_to_load',
                                                            'outcome2':'Wait_to_dump',
                                                            'outcome3':'Stuck',
                                                            'outcome4':'Lost',
                                                            'kill':'Kill'},
                                                remapping={'Drive_counter_in':'sm_counter',
                                                           'e_stop':'e_stop'})

#STATE WAIT_TO_LOAD

        smach.StateMachine.add('Wait_to_load',Wait_to_load(),transitions={'Full_Bucket':'Drive',
                                                                          'kill':'Kill'}, 
                                                             remapping={'counter_in':'sm_counter',
                                                                        'e_stop':'e_stop',
                                                                        'counter_out':'sm_counter'})

#STATE WAIT_TO_DUMP

        smach.StateMachine.add('Wait_to_dump', Wait_to_dump(),transitions={'Empty_Bucket':'Drive',
                                                                           'Sensors Lost':'Lost',
                                                                           'kill':'Kill'},
                                                              remapping={'counter_in':'sm_counter',
                                                                         'e_stop':'e_stop',
                                                                         'counter_out':'sm_counter'})

#STATE LOST

        smach.StateMachine.add('Lost', Lost(), transitions={'Minibot_lost':'Idle',
                                                            'kill':'Kill'}, 
                                               remapping={'counter_in':'sm_counter',
                                                          'e_stop':'e_stop',
                                                          'counter_out':'sm_counter'})

#STATE KILL

        smach.StateMachine.add('Kill', Kill(), transitions={'Kill':'end'})

#STATE STUCK

        smach.StateMachine.add('Stuck',Stuck(), transitions={'Stuck':'Drive',
                                                             'kill':'Kill'}, 
                                                remapping={'counter_in':'sm_counter',
                                                           'e_stop':'e_stop',
                                                           'counter_out':'sm_counter'})

    return sm_minibot

    # Execute SMACH plan
    # First you create a state machine sm
    # .....
    # Creating of state machine sm finished
    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    #sis.start()

    # Execute the state machine
    #outcome = sm_minibot.execute()
    # Wait for ctrl-c to stop the application
    #rospy.spin()
    #sis.stop()



if __name__ == '__main__':
    main()
