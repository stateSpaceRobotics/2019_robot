#I do not know what I am importing, please send help!
import roslib
import rospy
import smach
import smach_ros
import tf2_ros
from std_msgs.msg import Bool
from move_base_msgs.msg import *
from geometry_msgs.msg import Twist
#Seriously do not know what we are importing

#define state Idle
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['run', 'death'],
                             input_keys=['Idle_counter_in', 'e_stop'],
                             output_keys=['Idle_counter_out'])
    
    def execute(self):
        #Run the state
        return 'run'


#define state Kill
class Kill():
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['kill'])
                            
    #The Kill state ends the program
    def execute(self,userdata):
        return 'kill'


#define state Stuck
class Stuck():
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['stuck', 'death'],
                             in_keys=['e_stop'])
        
    def execute(self, userdata):
        #Check to see if the user wants to end
        if userdata.e_stop == True:
            return 'death'

        #Loop
        return 'stuck'


   

def run_andy_main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm_andy = smach.StateMachine(outcomes=['end'])
    sm_andy.userdata.sm_idle_counter = 0
    sm_andy.userdata.sm_move_counter = 0
    sm_andy.userdata.e_stop = False

    # Did not include coordinates yet, is blank
    sm_andy.userdata.move_array[[0.960, -0.280], [0.843, 0.537], [0.960, -0.280]]
    global move_index
    move_index = 0
    
    # Open the container
    with sm_andy:
        #Add states: 
        smach.StateMachine.add('Idle', Idle(),
                               transitions={'run':'Drive',
                                            'death':'Kill'},
                               remapping={'Idle_counter_in': 'sm_idle_counter',
                                          'Idle_counter_out': 'sm_idle_counter',
                                          'e_stop': 'e_stop'})

        #initial starting condition
        def drive_goal_andy():
            drive_goal = MoveBaseGoal()
            drive_goal.target_pose.header.frame_id = "/map"
            drive_goal.target_pose.header.stamp = rospy.get_rostime()

            drive_goal.target_pose.pose.position.x = 1
            drive_goal.target_pose.pose.position.y = 1
            drive_goal.target_pose.pose.orientation.w = 1
            drive_goal.target_pose.pose.orientation.z = 0

            return drive_goal

        smach.StateMachine.add('Drive',
                               smach_ros.SimpleActionState('/dumper/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb = drive_goal_andy,
                                                           input_keys=['']), #N/A
                               transitions={'motion':'Move1',
                                            'no_motion':'Stuck',
                                            'preempted': 'Drive'},
                               remapping={''}) #N/A

        #First of Four Movements
        def move1_goal_andy(self, userdata):
            move_goal = MoveBaseGoal()
            move_goal.target_pose.header.frame_id = "/map"
            move_goal.target_pose.header.stamp = rospy.get_rostime()

            #Figure out orientation and position during testing
            move_goal.target_pose.pose.position.x = 1
            move_goal.target_pose.pose.position.y = 2
            move_goal.target_pose.pose.orientation.w = 0.707
            move_goal.target_pose.pose.orientation.z = 0.707

            return move_goal

        smach.StateMachine.add('Move1',
                               smach_ros.SimpleActionState('/dumper/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb = move1_goal_andy,
                                                           input_keys=['aruco_pose']), #N/A
                               transitions={'succeeded':'Move2',
                                            'aborted':'Stuck',
                                            'preempted': 'Move1'},
                               remapping={'aruco_pose': 'userdata_aruco_pose'}) #N/A


        #Second of Four Movements
        def move2_goal_andy(self, userdata):
            move_goal = MoveBaseGoal()
            move_goal.target_pose.header.frame_id = "/map"
            move_goal.target_pose.header.stamp = rospy.get_rostime()

            #Figure out orientation and position during testing
            move_goal.target_pose.pose.position.x = 2
            move_goal.target_pose.pose.position.y = 2
            move_goal.target_pose.pose.orientation.w = 0
            move_goal.target_pose.pose.orientation.z = 1

            return move_goal

        smach.StateMachine.add('Move2',
                               smach_ros.SimpleActionState('/dumper/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb = move2_goal_andy,
                                                           input_keys=['aruco_pose']), #N/A
                               transitions={'succeeded':'Move3',
                                            'aborted':'Stuck',
                                            'preempted': 'Move2'},
                               remapping={'aruco_pose': 'userdata_aruco_pose'}) #N/A

        #Third of Four Movements
        def move3_goal_andy(self, userdata):
            move_goal = MoveBaseGoal()
            move_goal.target_pose.header.frame_id = "/map"
            move_goal.target_pose.header.stamp = rospy.get_rostime()

            #Figure out orientation and position during testing
            move_goal.target_pose.pose.position.x = 2
            move_goal.target_pose.pose.position.y = 1
            move_goal.target_pose.pose.orientation.w = -0.707
            move_goal.target_pose.pose.orientation.z = 0.707

            return move_goal

        smach.StateMachine.add('Move3',
                               smach_ros.SimpleActionState('/dumper/move_base_recovery',
                                                           MoveBaseAction,
                                                           goal_cb = move3_goal_andy,
                                                           input_keys=['aruco_pose']), #N/A
                               transitions={'succeeded':'Drive',
                                            'aborted':'Stuck',
                                            'preempted': 'Move3'},
                               remapping={'aruco_pose': 'userdata_aruco_pose'}) #N/A

        
        
        #Third of Four Movements
        
        # def move4_goal_andy(self, userdata):
        #     move_goal = MoveBaseGoal()
        #     move_goal.target_pose.header.frame_id = "/map"
        #     move_goal.target_pose.header.stamp = rospy.get_rostime()

        #     #Figure out orientation and position during testing
        #     move_goal.target_pose.pose.position.x = 1
        #     move_goal.target_pose.pose.position.y = 1
        #     move_goal.target_pose.pose.orientation.w = 0.707
        #     move_goal.target_pose.pose.orientation.z = 0.707

        #     return move_goal

        # smach.StateMachine.add('Move4',
        #                        smach_ros.SimpleActionState('/dumper/move_base_recovery',
        #                                                    MoveBaseAction,
        #                                                    goal_cb = move4_goal_andy,
        #                                                    input_keys=['aruco_pose']), #N/A
        #                        transitions={'succeeded':'Drive',
        #                                     'aborted':'Stuck',
        #                                     'preempted': 'Move4'},
        #                        remapping={'aruco_pose': 'userdata_aruco_pose'}) #N/A
        
        #-------------------------------------------------------
                               
        smach.StateMachine.add('Kill', Kill(),
                               transitions={'kill': 'end'})
                               
        smach.StateMachine.add('Stuck', Stuck(),
                               transitions={'stuck': 'Idle',
                                            'death': 'Kill'},
                               remapping={'e_stop': 'e_stop'})

        return sm_andy



if __name__ == "__main__":
    run_andy_main()