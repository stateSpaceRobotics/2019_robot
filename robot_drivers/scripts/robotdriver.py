#!/usr/bin/env python
import socket
import rospy
import math
from sensor_msgs.msg import Joy
from time import sleep
from random import gauss
from geometry_msgs.msg import Twist

#Wrapper class for the UDP Socket
class UDP_Handler:

    def __init__(self, ip, port):
        self.__ip = ip
        self.__port = port
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def sendMessage(self, message):
        self.__sock.sendto(message, (self.__ip, self.__port))


def sign(x):
    if x < 0.0:
        return -1.0
    return 1.0

def calculateVelocity(radius, time):
    return (2.0 * math.pi * radius) / time


def UDPSendMsg(twist_msg):
    #Build the message to send over UDP

    x = twist_msg.linear.x
    z = twist_msg.angular.z

    if z != 0.0:
        r = abs(x / z)
        t = abs((2.0 * math.pi) / z)

        l = 0.115

        rleft = r + sign(z) * sign(x) * -1.0 * l
        rright = r + sign(z) * sign(x) * l

        vleft = sign(x) * calculateVelocity(rleft, t)
        vright = sign(x) * calculateVelocity(rright, t)

        debug_message = "\nvl: " + str(vleft)
        debug_message += "\nvr: " + str(vright)
        debug_message += "\nr: " + str(r)
        debug_message += "\nt: " + str(t)
        debug_message += "\nrl: " + str(rleft)
        debug_message += "\nrr: " + str(rright)
        debug_message += "\nsign(x): " + str(sign(x))
        debug_message += "\nsign(z): " + str(sign(z)) + "\n"

        rospy.loginfo(debug_message)

    else:
        vleft = x
        vright = x

    vleft = vleft / .06
    vright = -1.0 * vright / .06


    message = str(FRONT_LEFT_DRIVE) + "," + str(vleft) + "\n"
    message += str(BACK_LEFT_DRIVE) + "," + str(vleft) + "\n"
    message += str(FRONT_RIGHT_DRIVE) + "," + str(vright) + "\n"
    message += str(BACK_RIGHT_DRIVE) + "," + str(vright) + "\n"
    # rospy.loginfo(message)
    set_point.sendMessage(message)

# def UDPSendMsg(twist_msg):
#     #Build the message to send over UDP

#     #Don't delete the old code or Andy will destroy you
#     x = twist_msg.linear.x
#     z = twist_msg.angular.z

#     if z != 0.0:
#         r = (x / z)

#         l = 0.115

#         lleft = -1 * l
#         lright = l

#         vleft = z*(r + lleft)
#         vright = z*(r + lright)

#         debug_message = "\nvl: " + str(vleft)
#         debug_message += "\nvr: " + str(vright)
#         debug_message += "\nr: " + str(r) + "\n"

#         rospy.loginfo(debug_message)

#     else:
#         vleft = x
#         vright = x


#     vleft = vleft / .06
#     vright = -1.0 * vright / .06


#     message = str(FRONT_LEFT_DRIVE) + "," + str(vleft) + "\n"
#     message += str(BACK_LEFT_DRIVE) + "," + str(vleft) + "\n"
#     message += str(FRONT_RIGHT_DRIVE) + "," + str(vright) + "\n"
#     message += str(BACK_RIGHT_DRIVE) + "," + str(vright) + "\n"
#     # rospy.loginfo(message)
#     set_point.sendMessage(message)



#Motor Indices on embedded side
FRONT_LEFT_DRIVE = 0
FRONT_RIGHT_DRIVE = 1
BACK_RIGHT_DRIVE = 2
BACK_LEFT_DRIVE = 3

#Values for drive motor value calculation
drive_motor_midpoint = 0
drive_motor_increment_value = 2 * math.pi
drive_motor_max_fraction = 1.0

set_point = UDP_Handler("192.168.0.32", 3233)

def main():

    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/dumper/cmd_vel", Twist, UDPSendMsg)

    rospy.spin()

if __name__ == '__main__':
    main()

