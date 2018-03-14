#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def move():
    # Starts a new node
    rospy.init_node('robot_control', anonymous=True)

    while not rospy.is_shutdown():
        print("-------------------------------------------")
        robot = raw_input("Which robot? (robot_red, sensor_tower, digger, robot_yellow: ")
        decision = robot + "/cmd_vel"
        velocity_publisher = rospy.Publisher(decision, Twist, queue_size=10)
        vel_msg = Twist()
        isRotate = input("Rotate? (True or False): ")
        if isRotate:
            speed = input("Angular speed (degrees/sec): ")
            angle = input('Angle (degrees): ')
            angular_speed = speed*2*PI/360
            relative_angle = angle*2*PI/360
            isClockWise = input("Clockwise? (True or False): ")
            if isClockWise:
                vel_msg.angular.z = -abs(angular_speed)
            else:
                vel_msg.angular.z = abs(angular_speed)
            vel_msg.linear.x = 0
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0

            t0 = rospy.Time.now().to_sec()
            current_angle = 0

            while(current_angle < relative_angle):
                velocity_publisher.publish(vel_msg)
                t1=rospy.Time.now().to_sec()
                current_angle= angular_speed*(t1-t0)
            #After the loop, stops the robot
            vel_msg.angular.z = 0
            #Force the robot to stop
            velocity_publisher.publish(vel_msg)
            #rospy.spin()
        elif isRotate != False:
            raise ValueError("Wrong input")

        isMove = input("Move? (True or False): ")
        if isMove:
            speed = input("Input your speed (m/s):")
            distance = input("Type your distance (m):")
            isForward = input("Foward? (True or False): ")#True or False

            #Checking if the movement is forward or backwards
            if(isForward):
                vel_msg.linear.x = abs(speed)
            else:
                vel_msg.linear.x = -abs(speed)
            #Since we are moving just in x-axis
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0

            #Setting the current time for distance calculus
            t0 = rospy.Time.now().to_sec()
            current_distance = 0

            #Loop to move the turtle in an specified distance
            while(current_distance < distance):
                #Publish the velocity
                velocity_publisher.publish(vel_msg)
                #Takes actual time to velocity calculus
                t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
                current_distance= speed*(t1-t0)
            #After the loop, stops the robot
            vel_msg.linear.x = 0
            #Force the robot to stop
            velocity_publisher.publish(vel_msg)
            #rospy.spin()
        elif isMove != False:
            raise ValueError("Wrong input")

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
