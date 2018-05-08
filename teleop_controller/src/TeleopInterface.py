#!/usr/bin/env python
import socket
import rospy
import math
from sensor_msgs.msg import Joy

#Wrapper class for the UDP Socket
class UDP_Handler:

    def __init__(self, ip, port):
        self.__ip = ip
        self.__port = port
        self.__sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def sendMessage(self, message):
        self.__sock.sendto(message, (self.__ip, self.__port))


#Base class for all Teleop Interfacing
class RobotInterface:

    #Static variables for robot selection
    interface_count = 0
    controller_interface_select = 0
    controller_interface_select_toggle_time = 0

    def __init__(self, ip):

        #Dictionaries for Controller State array accessing
        self.buttons = { "A" : 0, "B" : 1, "X" : 2, "Y" : 3, "LB" : 4, "RB" : 5, "BACK" : 6, "START" : 7, "POWER" : 8, "L3" : 9, "R3" : 10 }
        self.axes = { "LS_LEFT_RIGHT" : 0, "LS_UP_DOWN" : 1, "LT" : 2, "RS_LEFT_RIGHT" : 3, "RS_UP_DOWN" : 4, "RT" : 5, "DPAD_LEFT_RIGHT" : 6, "DPAD_UP_DOWN" : 7 }

        #Motor Indices on embedded side
        self.FRONT_LEFT_DRIVE = 1
        self.FRONT_RIGHT_DRIVE = 9
        self.BACK_RIGHT_DRIVE = 0
        self.BACK_LEFT_DRIVE = 8

        self.FRONT_LEFT_ACTUATOR = 6
        self.FRONT_RIGHT_ACTUATOR = 7
        self.BACK_LEFT_ACTUATOR = 5
        self.BACK_RIGHT_ACTUATOR = 4

        self.BUCKET_MOTOR = 2
        self.CONVEYOR_MOTOR = 3

        self.SENSOR_TOWER_BASE_MOTOR = 10
        self.SENSOR_TOWER_TOP_MOTOR = 11

        rospy.init_node('listener', anonymous = True)
        rospy.Subscriber("/joy", Joy, self.controllerCallback)

        RobotInterface.controller_interface_select_toggle_time = rospy.Time.now()

        #Motor set point UDP device
        self.set_point = UDP_Handler(ip, 3233)
        #self.set_point = UDP_Handler(ip, rospy.get_param("set_point_port"))
        #self.motor_state = UDP_Handler(ip, rospy.get_param("motor_state_port"))
        #self.tuning_gains = UDP_Handler(ip, rospy.get_param("tuning_gains_port"))

    def controllerCallback(self, controller_state):
        pass


#Inherited class for the digger bot
class DiggerInterface(RobotInterface):
    
    def __init__(self, ip):
        RobotInterface.__init__(self, ip)
        self.__my_name = "Digger Bot"
        self.__my_selector = RobotInterface.interface_count
        RobotInterface.interface_count += 1

        #Flags for toggles
        self.__is_digging = False
        self.__is_conveying = False
        self.__is_sifting = False
        self.__front_actuator_inversion = 1
        self.__back_actuator_inversion = 1

        #Timers used for Button "Debouncing"
        self.__back_actuator_toggle_time = rospy.Time.now()
        self.__front_actuator_toggle_time = rospy.Time.now()
        self.__digging_toggle_time = rospy.Time.now()
        self.__dig_speed_toggle_time = rospy.Time.now()
        self.__conveyor_toggle_time = rospy.Time.now() 
        self.__sifting_toggle_time = rospy.Time.now()    

        #Timer for alternating conveyor for sifting
        self.__sifting_toggle_direction_time = rospy.Time.now()   

        #Values for drive motor value calculation
        self.__drive_motor_midpoint = 511.0
        self.__drive_motor_increment_value = 171.0

        #Values for actuator value calculation
        self.__actuator_midpoint = 511.0
        self.__actuator_increment_value = 171.0

        #Values for bucket motor value calculation
        self.__bucket_motor_midpoint = 511.0
        self.__bucket_motor_regolith_speed = 85.0
        self.__bucket_motor_gravel_speed = 45.0
        self.__bucket_motor_increment_value = self.__bucket_motor_regolith_speed

        #Values for conveyor motor value calculation
        self.__conveyor_motor_midpoint = 511.0
        self.__conveyor_motor_increment_value = 171.0
        self.__conveyor_direction = 1

    #Callback override for the Joy controller Topic
    def controllerCallback(self, controller_state):

        #Handle conveyor sifting even if we're not currently controlling the digger
        if self.__is_sifting:
            duration = rospy.Time.now() - self.__sifting_toggle_direction_time

            if duration.to_sec() > 5:
                self.__conveyor_direction *= -1
                message = str(self.CONVEYOR_MOTOR) + "," + str(self.__conveyor_motor_midpoint + self.__conveyor_motor_increment_value * self.__is_conveying * self.__conveyor_direction) + "\n"
                self.set_point.sendMessage(message)
                self.__sifting_toggle_direction_time = rospy.Time.now()

        #If we're not currently trying to control this robot just return
        if self.__my_selector != RobotInterface.controller_interface_select:
            return

        #Change currently selected robot
        if controller_state.buttons[self.buttons["BACK"]]:
            duration = rospy.Time.now() - RobotInterface.controller_interface_select_toggle_time

            if duration.to_sec() > 1:
                RobotInterface.controller_interface_select += 1

                if RobotInterface.controller_interface_select >= RobotInterface.interface_count:
                    RobotInterface.controller_interface_select = 0

                RobotInterface.controller_interface_select_toggle_time = rospy.Time.now()

        #Handle changing of bucket speed for regolith/gravel
        if controller_state.axes[self.axes["DPAD_LEFT_RIGHT"]] != 0:
            duration = rospy.Time.now() - self.__dig_speed_toggle_time

            if duration.to_sec() > 1:
                if controller_state.axes[self.axes["DPAD_LEFT_RIGHT"]] == -1:
                    self.__bucket_motor_increment_value = self.__bucket_motor_regolith_speed
                else:
                    self.__bucket_motor_increment_value = self.__bucket_motor_gravel_speed
                
                self.__dig_speed_toggle_time = rospy.Time.now()

        #Activate the bucket ladder
        if controller_state.buttons[self.buttons["X"]]:
            duration = rospy.Time.now() - self.__digging_toggle_time

            if duration.to_sec() > 1:
                self.__is_digging = not self.__is_digging
                self.__digging_toggle_time = rospy.Time.now()

        #Invert the front actuators
        if controller_state.buttons[self.buttons["LB"]]:
            duration = rospy.Time.now() - self.__front_actuator_toggle_time

            if duration.to_sec() > 1:
                self.__front_actuator_inversion *= -1
                self.__front_actuator_toggle_time = rospy.Time.now()

        #Invert the back actuators
        if controller_state.buttons[self.buttons["RB"]]:
            duration = rospy.Time.now() - self.__back_actuator_toggle_time

            if duration.to_sec() > 1:
                self.__back_actuator_inversion *= -1
                self.__back_actuator_toggle_time = rospy.Time.now()

        #Toggle the conveyor belt
        if controller_state.buttons[self.buttons["B"]]:
            duration = rospy.Time.now() - self.__conveyor_toggle_time

            if duration.to_sec() > 1:
                self.__is_conveying = not self.__is_conveying
                self.__conveyor_toggle_time = rospy.Time.now()

        #Toggle conveyor belt sifting back and forth
        if controller_state.buttons[self.buttons["A"]]:
            duration = rospy.Time.now() - self.__sifting_toggle_time

            if duration.to_sec() > 1:
                self.__is_sifting = not self.__is_sifting

                if not self.__is_sifting:
                    self.__conveyor_direction = 1

                self.__sifting_toggle_time = rospy.Time.now()

        #Output Info
        rospy.loginfo("Selected Robot: " + str(self.__my_name) + "\n")
        rospy.loginfo("Is Digging: " + str(self.__is_digging) + "\n")
        rospy.loginfo("Is Conveying: " + str(self.__is_conveying) + "\n")
        rospy.loginfo("Is Sifting: " + str(self.__is_sifting) + "\n")
        rospy.loginfo("Front Actuators Inversion: " + str(self.__front_actuator_inversion) + "\n")
        rospy.loginfo("Back Actuators Inversion: " + str(self.__back_actuator_inversion) + "\n")

        #Build the message to send over UDP
        message = str(self.FRONT_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.FRONT_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"

        message += str(self.FRONT_LEFT_ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["LT"]] - 1) / -2) * self.__front_actuator_inversion) + "\n"
        message += str(self.FRONT_RIGHT_ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["LT"]] - 1) / -2) * self.__front_actuator_inversion) + "\n"
        message += str(self.BACK_LEFT_ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["RT"]] - 1) / -2) * self.__back_actuator_inversion) + "\n"
        message += str(self.BACK_RIGHT_ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["RT"]] - 1) / -2) * self.__back_actuator_inversion) + "\n"

        message += str(self.BUCKET_MOTOR) + "," + str(self.__bucket_motor_midpoint - self.__bucket_motor_increment_value * self.__is_digging) + "\n"
        message += str(self.CONVEYOR_MOTOR) + "," + str(self.__conveyor_motor_midpoint + self.__conveyor_motor_increment_value * self.__is_conveying * self.__conveyor_direction) + "\n"

        rospy.loginfo("\n\nmessage:\n" + message)

        self.set_point.sendMessage(message)


#Inherited class for the Sensor Tower
class DumperInterface(RobotInterface):

    def __init__(self, ip):
        RobotInterface.__init__(self, ip)
        self.__my_name = "Sensor Tower"
        self.__my_selector = RobotInterface.interface_count
        RobotInterface.interface_count += 1

        self.__drive_motor_midpoint = 511.0
        self.__drive_motor_increment_value = 171.0

        self.__arm_motors_midpoint = 511.0
        self.__arm_base_motor_increment_value = 171.0
        self.__arm_top_motor_increment_value = 171.0

    #Callback override for the Joy controller Topic
    def controllerCallback(self, controller_state):

        #If we're not currently trying to control this robot just return
        if self.__my_selector != RobotInterface.controller_interface_select:
           return

        #Change currently selected robot
        if controller_state.buttons[self.buttons["BACK"]]:
            duration = rospy.Time.now() - RobotInterface.controller_interface_select_toggle_time

            if duration.to_sec() > 1:
                RobotInterface.controller_interface_select += 1

                if RobotInterface.controller_interface_select >= RobotInterface.interface_count:
                    RobotInterface.controller_interface_select = 0

                RobotInterface.controller_interface_select_toggle_time = rospy.Time.now()
                

        rospy.loginfo("Selected Robot: " + str(self.__my_name))

        #Build the message to send over UDP
        message = str(self.FRONT_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.FRONT_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"

        message += str(self.SENSOR_TOWER_BASE_MOTOR) + "," + str(self.__arm_motors_midpoint + self.__arm_base_motor_increment_value * controller_state.axes[self.axes["RT"]]) + "\n"
        message += str(self.SENSOR_TOWER_TOP_MOTOR) + "," + str(self.__arm_motors_midpoint + self.__arm_top_motor_increment_value * controller_state.axes[self.axes["LT"]]) + "\n"

        rospy.loginfo("\n\nmessage:\n" + message)

        #self.set_point.sendMessage(message)


#Inherited class for the minibot
class TransporterInterface(RobotInterface):

    def __init__(self, ip):
        RobotInterface.__init__(self, ip)
        self.__my_name = "Mini Bot"
        self.__my_selector = RobotInterface.interface_count
        RobotInterface.interface_count += 1

        #Values for drive motor value calculation
        self.__drive_motor_midpoint = 511.0
        self.__drive_motor_increment_value = 171.0

    #Callback override for the Joy controller Topic
    def controllerCallback(self, controller_state):
        
        #If we're not currently trying to control this robot just return
        if self.__my_selector != RobotInterface.controller_interface_select:
           return

        #Change currently selected robot
        if controller_state.buttons[self.buttons["BACK"]]:
            duration = rospy.Time.now() - RobotInterface.controller_interface_select_toggle_time

            if duration.to_sec() > 1:
                RobotInterface.controller_interface_select += 1

                if RobotInterface.controller_interface_select >= RobotInterface.interface_count:
                    RobotInterface.controller_interface_select = 0

                RobotInterface.controller_interface_select_toggle_time = rospy.Time.now()
                

        rospy.loginfo("Selected Robot: " + str(self.__my_name) + "\n")

        #Build the message to send over UDP
        message = str(self.FRONT_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.FRONT_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"

        rospy.loginfo("\n\nmessage:\n" + message)

        #self.set_point.sendMessage(message)


def main():
    minibot = TransporterInterface("ip")
    digger = DiggerInterface("192.168.0.101")
    sensorTower = DumperInterface("ip")

    rospy.spin()

if __name__ == '__main__':
    main()