#!/usr/bin/env python
import socket
import rospy
import math
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


#Build the message of motor commands to send over UDP
def UDPSendMsg(twist_msg):
    x = twist_msg.linear.x
    z = twist_msg.angular.z

    #If there's no angular component (z == 0) the motors just receive the linear component
    Vright = x
    Vleft = x

    wheelOffset = 0.13335
    wheelRadius = 0.0762

    #If there is an angular component we have to calculate the velocity for the left and right side motors to achieve the desired arc
    if z != 0:
        r = abs(x/z)
        zabs = abs(z)
        signx = sign(x)
        signz = sign(z)

        #Calculate the radii for the arcs followed by the right side and the left side
        rright = r + signx * signz * wheelOffset
        rleft = r + signx * signz * -1.0 * wheelOffset        

        #Calculate the linear velocity necessary to achieve the corresponding arcs
        Vright = signx * zabs * rright
        Vleft = signx * zabs * rleft
 
    #Convert the calculated linear velocities into angular velocities because that is what the embedded side wants
    Vright = -1.0 * Vright / wheelRadius    #Note that the right side is multiplied by -1. This is because direction is flipped due to the motor's physical orientation
    Vleft = Vleft / wheelRadius
    
    #Construct and send the message to the embedded side via UDP
    message = str(FRONT_LEFT_DRIVE) + "," + str(Vleft) + "\n"
    message += str(BACK_LEFT_DRIVE) + "," + str(Vleft) + "\n"
    message += str(FRONT_RIGHT_DRIVE) + "," + str(Vright) + "\n"
    message += str(BACK_RIGHT_DRIVE) + "," + str(Vright) + "\n"
    rospy.loginfo(message)
    set_point.sendMessage(message)

# def UDPSendMsg(twist_msg):
#     x = twist_msg.linear.x
#     z = twist_msg.angular.z

    # if z != 0.0:
    #     r = abs(x / z)
    #     t = abs((2.0 * math.pi) / z)

    #     l = 0.115

    #     rleft = r + sign(z) * sign(x) * -1.0 * l
    #     rright = r + sign(z) * sign(x) * l

    #     vleft = sign(x) * calculateVelocity(rleft, t)
    #     vright = sign(x) * calculateVelocity(rright, t)

    #     debug_message = "\nvl: " + str(vleft)
    #     debug_message += "\nvr: " + str(vright)
    #     debug_message += "\nr: " + str(r)
    #     debug_message += "\nt: " + str(t)
    #     debug_message += "\nrl: " + str(rleft)
    #     debug_message += "\nrr: " + str(rright)
    #     debug_message += "\nsign(x): " + str(sign(x))
    #     debug_message += "\nsign(z): " + str(sign(z)) + "\n"

    #     rospy.loginfo(debug_message)

    # else:
    #     vleft = x
    #     vright = x

    # vleft = vleft / .06
    # vright = -1.0 * vright / .06

    # message = str(FRONT_LEFT_DRIVE) + "," + str(vleft) + "\n"
    # message += str(BACK_LEFT_DRIVE) + "," + str(vleft) + "\n"
    # message += str(FRONT_RIGHT_DRIVE) + "," + str(vright) + "\n"
    # message += str(BACK_RIGHT_DRIVE) + "," + str(vright) + "\n"
    # # rospy.loginfo(message)
    # set_point.sendMessage(message)

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

set_point = UDP_Handler("192.168.0.32", 3233)

def main():
    rospy.init_node('listener', anonymous = True)
    rospy.Subscriber("/dumper/cmd_vel", Twist, UDPSendMsg)

    rospy.spin()

if __name__ == '__main__':
    main()

