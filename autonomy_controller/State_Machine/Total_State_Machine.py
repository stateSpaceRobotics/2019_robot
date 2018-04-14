#!/usr/bin/env python
# The state machine of the Diggerbot

import roslib
import rospy
import smach
import smach_ros
import SensorTower_State_Machine
import Minibot_State_Machine
import Diggerbot_State_Machine
# define state Idle
#


class Startup(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['begin', 'kill'],
                             input_keys=['e_stop'])

    def execute(self, userdata):
        #rospy.loginfo('Executing state Idle')
        if userdata.e_stop == True:
            return 'kill'
        return 'begin'


def main():
    rospy.init_node('smach_example_state_machine')
    global sm
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0
    sm.userdata.e_stop = False
    sm.userdata.dumper_in_pos = False
    sm.userdata.digger_in_pos = False
    sm.userdata.transporter_ready_to_dump = False
    sm.userdata.transporter_ready_to_load = False

    sm_sensortower = SensorTower_State_Machine.sensortower_main()
    sm_diggerbot = Diggerbot_State_Machine.diggerbot_main()
    sm_minibot = Minibot_State_Machine.minibot_main()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Startup', Startup(),
                               transitions={'begin': 'CON',
                                            'kill': 'outcome4'},
                               remapping={'e_stop': 'e_stop'})
        sm_con = smach.Concurrence(outcomes=['loop', 'end'],
                                   default_outcome='loop',
                                   outcome_map={'end':
                                                {'SensorTower': 'end',
                                                 'Minibot': 'end',
                                                 'Diggerbot': 'end'}})
        with sm_con:
            smach.Concurrence.add('SensorTower', sm_sensortower)
            smach.Concurrence.add('Diggerbot', sm_diggerbot)
            smach.Concurrence.add('Minibot', sm_minibot)
        smach.StateMachine.add('CON', sm_con,
                               transitions={'loop': 'CON',
                                            'end': 'outcome4'})

    # Execute SMACH plan
    # Execute SMACH plan
    # First you create a state machine sm
    # .....
    # Creating of state machine sm finished
    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    # Execute the state machine
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    # sis.stop()
    outcome.stop()


if __name__ == '__main__':
    main()
