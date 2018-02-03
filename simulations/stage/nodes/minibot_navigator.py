#!/usr/bin/env python

import rospy, signal, atexit, math
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose, PoseArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Bool
'''
This module is responsible for sending Twist commands to robot.
Input: Waypoints, Map
Output: Twist commands
'''
######################################
# Global constants
GOAL_FORCE_CONST    = 1.0  # magnitude used when calculating goal force
ANGULAR_SPEED       = 1.0
LINEAR_SPEED        = 0.8
RYAN_CONSTANT       = 7
GOAL_THRESH         = 0.1       # radius around goal that it's okay to stop in 
######################################
# Load global topic names from ros params
######################################
DRIVE_TOPIC = rospy.get_param("topics/drive_cmds", "/cmd_vel")
ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "/odom")
# DRIVE_TOPIC = rospy.get_param("topics/drive_cmds", "/r3d1_diff_drive_controller/cmd_vel")
# ROBOPOSE_TOPIC = rospy.get_param("topics/localization_pose", "/r3d1/odom")
RUNTYPE = rospy.get_param("/settings/run_type","default")

class PFieldNavigator(object):

    def __init__(self):
        '''
        Potential field navigator constructor.
        '''
        rospy.init_node("minibot_pfield")
        self.robot_pose = Pose()
        self.others_points = []
        self.received_pose = False
        self.current_goal = Point()
        self.centroid_obstacles = None

        self.previousDirection = 1.0 #initalize it to move forward more often
        
        ######################################
        # Setup ROS publishers
        ######################################
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, Twist, queue_size = 10)

        ######################################
        # Setup ROS subscibers
        ######################################
        rospy.Subscriber(ROBOPOSE_TOPIC, Odometry, self.robot_pose_odom_callback)
        rospy.Subscriber('/obstacle_centroids', PointCloud, self.obstacle_callback)
        rospy.Subscriber('/miniBot_poses', PoseArray, self.minibot_poses_callback)

    def minibot_poses_callback(self, data):
        self.others_points = []
        for each in data.poses:
            self.others_points.append(each.position)

    def robot_pose_callback(self, data):
        '''
        Callback for robot localization pose.
        '''
        self.received_pose = True
        self.robot_pose = self.transform_pose(data.pose)

    def robot_pose_odom_callback(self, data):
        '''
        Callback for robot localization pose from odom.
        '''
        self.received_pose = True
        self.robot_pose = data.pose.pose
        
    def obstacle_callback(self, pc):
        self.centroid_obstacles = pc

    def transform_pose(self, pose):
        '''
        Given a global pose, transform to local robot pose
        '''
        # convert quat orientation to eulers
        global_orient = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]) 
        
        roll = global_orient[0]
        pitch = global_orient[1]
        yaw = global_orient[2]

        local_yaw = yaw - math.pi / 2

        robot_orient = quaternion_from_euler(roll, pitch, local_yaw)

        local_pose = pose
        local_pose.orientation.x = robot_orient[0]
        local_pose.orientation.y = robot_orient[1]
        local_pose.orientation.z = robot_orient[2]
        local_pose.orientation.w = robot_orient[3]

        return local_pose

    def run(self):
        '''
        '''
        rate = rospy.Rate(10)
        # hold up for some messages
        rospy.wait_for_message(ROBOPOSE_TOPIC, PoseStamped)
        
        # work hard doing good stuff
        while not rospy.is_shutdown():

            temp_obstacles = []
            # temp_obstacles.extend(self.centroid_obstacles)

        	#New pose
            if self.received_pose:
                self.received_pose = False
                # grab current goal and pose information
                nav_goal = self.current_goal
                robot_pose = self.robot_pose 
                print("==============================")
                print("Navigating...")
                # print(" ** Position: \n" + str(robot_pose.position))
                print(" ** Position: \n" + str(robot_pose))

                # Calculate force vector for its position
                map_force = pFieldMinibot(self.robot_pose)
                print("Map force: " + str(map_force))
                # Calculate repulsive force
                repulsive_force = self.calc_repulsive_force(temp_obstacles, robot_pose)

                #combine the forces
                combine_force = [map_force[0]+repulsive_force[0], map_force[1]+repulsive_force[1]]

                # Get final drive vector (goal, obstacle forces)
                # Calculate twist message from drive vector
                drive_cmd = self.drive_from_force(combine_force, robot_pose)
                self.drive_pub.publish(drive_cmd)
            else:
                pass

            rate.sleep()

    def at_goal(self, robot_pose, goal):
        '''
        Given a robot_pose and a goal coordinate, this node determines if robot is at the goal 
        '''
        # calc distance 
        dist = math.sqrt((goal.x - robot_pose.position.x)**2 + (goal.y - robot_pose.position.y)**2)
        at_goal = True if dist <= GOAL_THRESH else False
        return at_goal


    def drive_from_force(self, force, robot_pose):
        '''
        Given a force vector, generate Twist message 
        '''
        cmd = Twist()
        max_angle = math.pi / 2.0
        spin_thresh = math.pi / 4.0 #I added the / 4.0 to see if it helps the back up
        # get force magnitude
        force_mag = math.hypot(force[0], force[1])
        if force_mag == 0: return cmd
        # normalize force
        force[0] = force[0] / float(force_mag)
        force[1] = force[1] / float(force_mag)
        # convert quat orientation to eulers
        robot_orient = euler_from_quaternion([robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w]) 
        # Get force angle (in global space)
        force_angle = math.atan2(force[1], force[0])
        # put force angle in robot space
        force_angle = -1 * (force_angle - (math.pi / 2.0))

        #this is for when the force is behind the robot
        signed_lin_vel = LINEAR_SPEED * abs(2*math.cos(force_angle)-math.sin(force_angle))  #this number needs to be changed, we don't want it to move forward quickly while turning, so it should be affected by angular velocity

        if abs(force_angle)!=0 and abs(force_angle) > math.radians(90 + self.previousDirection * 180):#180):#45):#previousDirection math makes it more likly to back up again if it was just backing up
        	#pi-|angle| gives you the magnitude of the angle
        	# angle/|angle| will be the sign of the original angle
        	force_angle = (math.pi - abs(force_angle)) * (-1 * (force_angle / abs(force_angle)))
        	signed_lin_vel *= -1.0
        	self.previousDirection = -1.0
        else:
        	self.previousDirection = 1.0

        # get difference to robot's current yaw
        angle_diff = self.wrap_angle(force_angle - robot_orient[2])
        print("Robot Yaw: " + str(math.degrees(robot_orient[2])))
        print("Force angle: " + str(math.degrees(force_angle)))
        print("Force Magnitude: " + str(force_mag))
        print("Angle diff: " + str(math.degrees(angle_diff)))

        ang_vel = (angle_diff / max_angle) * ANGULAR_SPEED
        lin_vel = 0 if abs(angle_diff) >= spin_thresh else signed_lin_vel

        vel = self._transform_for_ryan(ang_vel, lin_vel)

        print("Ang vel: " + str(vel[0]))
        print("Lin Vel: " + str(vel[1]))
        cmd.angular.z = vel[0]
        cmd.linear.x = vel[1]

        return cmd

    def _transform_for_ryan(self, ang_vel, lin_vel):
        '''
        Given angular velocity and linear velocity, transform for ryan (-7, 7)
        Contact: Ryan Smith, (228) 623 - 9492
        '''
        # Normalize, then multiply by seven
        mag = math.sqrt(ang_vel**2 + lin_vel**2)
        ang_vel /= mag
        lin_vel /= mag
        return [ang_vel * RYAN_CONSTANT, lin_vel * RYAN_CONSTANT]


    def calc_goal_force(self, nav_goal, robot_pose):
        '''
        given a goal point and a robot pose, calculate and return x and y components of goal force
        '''
        FIELD_SPREAD = 10.0 # radius around goal where pfield is scaled
        ALPHA = 1.0
        # get distance between goal and robot
        dist = math.sqrt((nav_goal.x - robot_pose.position.x)**2 + (nav_goal.y - robot_pose.position.y)**2)
        # get angle to goal
        angle_to_goal = math.atan2(nav_goal.y - robot_pose.position.y, nav_goal.x - robot_pose.position.x)
        # get force angle
        force_angle = self.wrap_angle(angle_to_goal)
        # math the components
        if dist < GOAL_THRESH:
            d_x = 0
            d_y = 0
        elif (GOAL_THRESH <= dist) and (dist <= FIELD_SPREAD + GOAL_THRESH):
            d_x = ALPHA * (dist - GOAL_THRESH) * math.cos(force_angle)
            d_y = ALPHA * (dist - GOAL_THRESH) * math.sin(force_angle)
        else: #dist > (FIELD_SPREAD + GOAL_THRESH)
            d_x = ALPHA * FIELD_SPREAD * math.cos(force_angle)
            d_y = ALPHA * FIELD_SPREAD * math.sin(force_angle)

        return [d_x, d_y]

    def calc_repulsive_force_others(self, robot_pose):
        rep_force = [0,0]
        CLOSETHRESH = 0.55
        MIDTHRESH = 1
        LARGE_NUMBER = 20
        BETA = 2

        myX = robot_pose.position.x
        myY = robot_pose.position.y
        for minibot in self.others_points:
            theirX = minibot.x
            theirY = minibot.y
            dist = math.sqrt((theirX - myX)**2 + (theirY - myY)**2)
            angle = math.atan2(theirY - myY, theirX - myX)

            if dist < CLOSETHRESH:
                d_x = LARGE_NUMBER if math.cos(angle) < 0.0 else -LARGE_NUMBER
                d_y = LARGE_NUMBER if math.sin(angle) < 0.0 else -LARGE_NUMBER

            elif dist < MIDTHRESH:
                d_x = -BETA * ((MIDTHRESH - dist)/MIDTHRESH) * math.cos(angle)
                d_y = -BETA * ((MIDTHRESH - dist)/MIDTHRESH) * math.sin(angle)
            else:
                d_x = 0
                d_y = 0

            rep_force[0] += d_x
            rep_force[1] += d_y
        return rep_force


    def calc_repulsive_force(self, obstacles, robot_pose):
        '''
        Given a list of obstacles and robot pose, calculate repulsive force
        '''
        # rep_force = [0, 0]
        rep_force = self.calc_repulsive_force_others(robot_pose)
        OBST_THRESH = 0.02
        FIELD_SPREAD = 1
        LARGE_NUMBER = 99
        BETA = 1.0          # scale factor
        for obstacle in obstacles:
            # calculate distance between robot and obstacle
            dist = math.sqrt((obstacle[0] - robot_pose.position.x)**2 + (obstacle[1] - robot_pose.position.y)**2)
            # calculate angle between robot and obstacle
            angle_to_obs = math.atan2(obstacle[1] - robot_pose.position.y, obstacle[0] - robot_pose.position.x)
            if dist < OBST_THRESH:
                # too close
                d_x = LARGE_NUMBER if math.cos(angle_to_obs) < 0.0 else -LARGE_NUMBER
                d_y = LARGE_NUMBER if math.sin(angle_to_obs) < 0.0 else -LARGE_NUMBER
            elif (OBST_THRESH <= dist) and (dist <= FIELD_SPREAD + OBST_THRESH):
                # field is scaled
                d_x = -BETA * (FIELD_SPREAD + OBST_THRESH - dist) * math.cos(angle_to_obs)
                d_y = -BETA * (FIELD_SPREAD + OBST_THRESH - dist) * math.sin(angle_to_obs)
            else:
                # obstacle is far away, don't care about it
                d_x = 0
                d_y = 0
            rep_force[0] += d_x
            rep_force[1] += d_y

        if RUNTYPE != "default":
            rep_force.reverse() #debug

        return rep_force



    def wrap_angle(self, angle):
        #This function will take any angle and wrap it into the range [-pi, pi]
        while angle >= math.pi:
            angle = angle - 2 * math.pi        
        while angle <= -math.pi:
            angle = angle + 2 * math.pi
        return angle

def pFieldMinibot(robot_pose):
    x = robot_pose.position.x
    y = robot_pose.position.y
    global RUNTYPE
    RUNTYPE = "stage" #DEBUG, force type

    if RUNTYPE == "default":
        return pFieldMinibotXY(x,y)
    elif RUNTYPE == "stage":
        force = pFieldMinibotXY(x,y)
        force.reverse()
        return force
    elif RUNTYPE == "gazebo":
        force = pFieldMinibotXY(x,y)
        #in gazebo, the x and y forces are reversed
        force.reverse()
        return force

    else:
        return pFieldMinibotXY(x,y)


yLength = 7.38
xLength = 3.78
xWallDist = 0.39
yWallDist = 0.38
xMiddleLength = 0.5
yMinningMainLen = 2.0
yMiningMidLen = 0.25
xDumpLength = 1.5
yDumpLength = 1.0
ySlantDumpTop = 1.5
ySlantDumpBot = 1.0
ySlantSlope = 3

def inLeftTri(x,y):
    if x >= xWallDist and x < (xLength/2 - xDumpLength/2) and y < ySlantDumpBot:
        yPointOnLine = (x - xWallDist)*(-1*ySlantSlope) + ySlantDumpBot
        return y >= yPointOnLine
    return False


def inRightTri(x,y):
    if x >= (xLength/2 + xMiddleLength/2) and x < (xLength - xWallDist) and y < ySlantDumpBot:
        yPointOnLine = (x - (xLength - xWallDist))*ySlantSlope  + ySlantDumpBot
        return y >= yPointOnLine
    return False

def pFieldMinibotXY(x,y):
    force = [0,0]

    print("x: "+str(x)+" y: "+str(y))

    #too close to top  or bottom wall
    if y >= (yLength -yWallDist):
        print("top wall")
        force[1] += -1
    elif y <= yWallDist:
        print("bottom wall")
        force[1] += 1

    if y <= yWallDist*2:
        print("Bottom wall 2")
        force[1] += 0.5

    #too close to left or right wall
    if x >= (xLength - xWallDist):
        print("right wall")
        force[0] += -0.7
    elif x <= xWallDist:
        print("left wall")
        force[0] += 0.7

    if x >= (xLength - 2*xWallDist):
        print("right wall")
        force[0] += -0.6
    elif x <= 2*xWallDist:
        print("left wall")
        force[0] += 0.6

    #This is for in the mining area
    if (x > (xWallDist + 2*xWallDist) and x < (xLength - xWallDist) and y >= (yLength - yWallDist - yMinningMainLen) and y < (yLength - yWallDist)):
        print("mining area")
        force[0] += -0.8
    elif (x >= (xLength/2 - xMiddleLength/2) and x <= (xLength/2 + xMiddleLength/2) and y >= (yLength - yWallDist - yMinningMainLen - yMiningMidLen) and y < (yLength - yWallDist - yMinningMainLen)):
        print("mining area extension")
        force[0] += -1

    #the middle, to help avoid the minibots on each side of loop crossing
    if x >= (xLength/2 - xMiddleLength/2) and x < (xLength/2) and y >= yDumpLength and y <= (yLength - yWallDist - yMinningMainLen - yMiningMidLen):
        print("Left middle")
        force[0] += -1
    elif x >= (xLength/2) and x <= (xLength/2 - xMiddleLength/2) and y >= yDumpLength and y <= (yLength - yWallDist - yMinningMainLen - yMiningMidLen):
        print("Right middle")
        force[0] += 1

    #Special case for dumping, not final
    if x >= (xLength/2 - xDumpLength/2) and x <= (xLength/2 + xDumpLength/2) and y <= yDumpLength:
        print("dumping area")
        force[0] += 1

    #the two columns of down of the left and up on the right
    elif x >= xWallDist and x < (xLength/2 - xMiddleLength/2) and y > ySlantDumpTop:# and y < (yLength - yWallDist - yMinningMainLen):
        print("left down")
        force[1] += -1
    elif x >= (xLength/2 + xMiddleLength/2) and x <= (xLength - xWallDist) and y > ySlantDumpTop and y < (yLength - yWallDist - yMinningMainLen + 0.5):
        print("right up")
        force[1] += 1

    #left angled to the dumping
    elif (x >= xWallDist and x < (xLength/2 - xMiddleLength/2) and y > ySlantDumpBot and y < ySlantDumpTop):
        print("left slanted down block")
        force[0] += 1
        force[1] += -1
    elif inLeftTri(x,y):
        print("left slanted down triangle")
        force[0] += 1
        force[1] += -1
    elif x >= xWallDist and x < (xLength/2 - xDumpLength/2) and y <= ySlantDumpBot:
        print("below left triangle")
        force[0] = 0
        force[1] = 1

    #right angled to dumping
    elif (x >= (xLength/2 + xMiddleLength/2) and x < (xLength - xWallDist) and y > ySlantDumpBot and y < ySlantDumpTop):
        print("right slanted down block")
        force[0] += 0.2
        force[1] += 1
    elif (inRightTri(x,y)):
        print("right slanted down triangle")
        force[0] += 0.3
        force[1] += 1
    elif x >= (xLength/2 + xMiddleLength/2) and x < (xLength - xWallDist) and y <= ySlantDumpBot:
        print("below right triangle")
        force[0] = 0
        force[1] = 1
    
    print(force)

    # force[0] = int(raw_input("x:"))
    # force[1] = int(raw_input("y:"))

    return force

def pFieldMinibotXY_original(x,y):
    force = [0,0]

    #TEMP THING DO NOT KEEP DEBUG
    # xtemp = abs(y)
    # y = abs(x)
    # x = xtemp
    print("x: "+str(x)+" y: "+str(y))
    #too close to top  or bottom wall
    if y >= 6.5:
        print("top wall")
        force[1] = -5
    elif y <= 0.5:
        print("bottom wall")
        force[1] = 5

    #too close to left or right wall
    if x >= 3.39:
        print("right wall")
        force[0] = -5
    elif x <= 0.39:
        print("left wall")
        force[0] = 5

    #This is for in the mining area
    #   consider changing it to the trapezoid in the image
    if x > 1 and x < 3.39 and y >= 5 and y < 7:
        print("mining area")
        force[0] += -1

    #the middle, to help avoid the minibots on each side of loop crossing
    if x >= 1.44 and x < 1.89 and y > 1 and y <= 5:
        print("Left middle")
        force[0] += -0.5
    elif x >= 1.89 and x <= 2.64 and y > 1 and y <= 5:
        print("Right middle")
        force[0] += 0.5

    #Special case for dumping, not final
    # if x >= 1.44 and x <= 2.64 and y > 0.2 and y<= 1:
    if x >= 0.39 and x <= 2.64 and y > 0.2 and y<= 1:
        print("dumping area")
        force[0] += 1
        # force[1] = 0

    #the two columns of down of the left and up on the right
    #   making both elif instead of 1 if and 1 elif 
    #   b/c special case for dump
    if x >= 0.39 and x < 1.89 and y > 1 and y < 7:
        print("left down")
        force[1] += -1
    elif x >= 1.89 and x <= 3.5 and y > 0.2 and y < 5.0:
        print("right up")
        force[1] += 1
    
    #TEMP FOR DEBUGGING STAGE, IDK WHY NECESSARY, STAGE IS WEIRD
    # force[0] = force[0]*-1
    # force[1] = force[1]*-1
    # force.reverse()
    print(force)
    # force = [0,0]

    return force

if __name__ == "__main__":
    navigator = PFieldNavigator()
    navigator.run()
