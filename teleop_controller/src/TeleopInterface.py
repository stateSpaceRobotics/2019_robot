#!/usr/bin/env python
import socket
import rospy
import math
from sensor_msgs.msg import Joy
from time import sleep
from random import gauss

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
    controller1_interface_select = 0
    controller2_interface_select = 4
    controller1_interface_select_toggle_time = 0
    controller2_interface_select_toggle_time = 0
    artificial_delay_enabled = False

    controller1_outstring = ""
    controller2_outstring = ""

    controller1_outstring_changed = True
    controller2_outstring_changed = True

    def __init__(self, ip):

        #Dictionaries for Controller State array accessing
        self.buttons =  { "A" : 0, "B" : 1, "X" : 2, "Y" : 3, "LB" : 4, "RB" : 5, "BACK" : 6, "START" : 7, "POWER" : 8, "L3" : 9, "R3" : 10 }
        self.axes =     { "LS_LEFT_RIGHT" : 0, "LS_UP_DOWN" : 1, "LT" : 2, "RS_LEFT_RIGHT" : 3, "RS_UP_DOWN" : 4, "RT" : 5, "DPAD_LEFT_RIGHT" : 6, "DPAD_UP_DOWN" : 7 }

        #values for gaussian sampling for artificial latency
        self.mu = 0.5
        self.sigma = 0.5

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
        rospy.Subscriber("/joy1", Joy, self.controllerCallback1)

        rospy.init_node('listener', anonymous = True)
        rospy.Subscriber("/joy2", Joy, self.controllerCallback2)

        RobotInterface.controller1_interface_select_toggle_time = rospy.Time.now()
        RobotInterface.controller2_interface_select_toggle_time = rospy.Time.now()

        #Motor set point UDP device
        self.set_point = UDP_Handler(ip, 3233)
        #self.set_point = UDP_Handler(ip, rospy.get_param("set_point_port"))
        #self.motor_state = UDP_Handler(ip, rospy.get_param("motor_state_port"))
        #self.tuning_gains = UDP_Handler(ip, rospy.get_param("tuning_gains_port"))

    def controllerCallback1(self, controller_state):
        self.handleControllerCallback(controller_state, 1)

    def controllerCallback2(self, controller_state):
        self.handleControllerCallback(controller_state, 2)

    def handleControllerCallback(self, controller_state, controller_id):
        pass


#Inherited class for the digger bot
class DiggerInterface(RobotInterface):
    
    def __init__(self, ip):
        RobotInterface.__init__(self, ip)
        self.__my_name = "Digger Bot"
        self.__my_selector = RobotInterface.interface_count
        RobotInterface.interface_count += 1

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
        self.__bucket_motor_midpoint = 511.0 #0
        self.__bucket_motor_speed = 85.0 #math.pi
        self.__bucket_motor_increment_value = 5.0
        self.__bucket_motor_max_speed = 171.0
        self.__bucket_motor_min_speed = 0

        #Values for conveyor motor value calculation
        self.__conveyor_motor_midpoint = 511.0
        self.__conveyor_motor_increment_value = 171.0
        self.__conveyor_direction = 1


    #Callback override for the Joy controller Topic
    def handleControllerCallback(self, controller_state, controller_id):

        #Handle conveyor sifting even if we're not currently controlling the digger
        if self.__is_sifting:
            duration = rospy.Time.now() - self.__sifting_toggle_direction_time

            if duration.to_sec() > 5:
                self.__conveyor_direction *= -1
                message = str(self.CONVEYOR_MOTOR) + "," + str(self.__conveyor_motor_midpoint + self.__conveyor_motor_increment_value * self.__is_conveying * self.__conveyor_direction) + "\n"
                self.set_point.sendMessage(message)
                self.__sifting_toggle_direction_time = rospy.Time.now()

        #If we're not currently trying to control this robot just return       
        if controller_id == 1: 
            if self.__my_selector != RobotInterface.controller1_interface_select:
                return
        else:
            if self.__my_selector != RobotInterface.controller2_interface_select:
                return

        #Change currently selected robot
        if controller_state.buttons[self.buttons["BACK"]]:
            if controller_id == 1:
                duration = rospy.Time.now() - RobotInterface.controller1_interface_select_toggle_time

                if duration.to_sec() > 1:
                    RobotInterface.controller1_interface_select += 1
                        
                    RobotInterface.controller1_interface_select = RobotInterface.controller1_interface_select % (RobotInterface.interface_count)

                    if RobotInterface.controller1_interface_select == RobotInterface.controller2_interface_select:
                        RobotInterface.controller1_interface_select += 1

                    RobotInterface.controller1_interface_select = RobotInterface.controller1_interface_select % (RobotInterface.interface_count)

                    RobotInterface.controller1_interface_select_toggle_time = rospy.Time.now()

                    RobotInterface.controller1_outstring_changed = True

            else:
                duration = rospy.Time.now() - RobotInterface.controller2_interface_select_toggle_time

                if duration.to_sec() > 1:
                    RobotInterface.controller2_interface_select += 1

                    RobotInterface.controller2_interface_select = RobotInterface.controller2_interface_select % (RobotInterface.interface_count)

                    if RobotInterface.controller2_interface_select == RobotInterface.controller1_interface_select:
                        RobotInterface.controller2_interface_select += 1

                    RobotInterface.controller2_interface_select = RobotInterface.controller2_interface_select % (RobotInterface.interface_count)

                    RobotInterface.controller2_interface_select_toggle_time = rospy.Time.now()

                    RobotInterface.controller2_outstring_changed = True

            return

        #Handle changing of bucket speed
        if controller_state.axes[self.axes["DPAD_LEFT_RIGHT"]] != 0:
            duration = rospy.Time.now() - self.__dig_speed_toggle_time

            if duration.to_sec() > 0.250:
                if controller_state.axes[self.axes["DPAD_LEFT_RIGHT"]] == -1:
                    self.__bucket_motor_speed += self.__bucket_motor_increment_value
                    if self.__bucket_motor_speed > self.__bucket_motor_max_speed:
                        self.__bucket_motor_speed = self.__bucket_motor_max_speed

                else:
                    self.__bucket_motor_speed -= self.__bucket_motor_increment_value
                    if self.__bucket_motor_speed < self.__bucket_motor_min_speed:
                        self.__bucket_motor_speed = self.__bucket_motor_min_speed
                
                self.__dig_speed_toggle_time = rospy.Time.now()

                if controller_id == 1:
                    RobotInterface.controller1_outstring_changed = True
                else:
                    RobotInterface.controller2_outstring_changed = True

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
        #rospy.loginfo("Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name))
        #rospy.loginfo("Is Digging: " + str(self.__is_digging))
        #rospy.loginfo("Is Conveying: " + str(self.__is_conveying))
        #rospy.loginfo("Is Sifting: " + str(self.__is_sifting))
        #rospy.loginfo("Front Actuators Inversion: " + str(self.__front_actuator_inversion))
        #rospy.loginfo("Back Actuators Inversion: " + str(self.__back_actuator_inversion))

        for button in controller_state.buttons:
            if button:
                if controller_id == 1:
                    RobotInterface.controller1_outstring_changed = True
                else:
                    RobotInterface.controller2_outstring_changed = True

        if controller_id == 1:
            if RobotInterface.controller1_outstring_changed:
                RobotInterface.controller1_outstring_changed = False
                RobotInterface.controller1_outstring = "Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name) + "\nIs Digging: " + str(self.__is_digging) + "\nIs Conveying: " + str(self.__is_conveying) + "\nIs Sifting: " + str(self.__is_sifting) + "\nFront Actuators Inversion: " + str(self.__front_actuator_inversion) + "\nBack Actuators Inversion: " + str(self.__back_actuator_inversion) + "\nCurrent Digging Speed: " + str(self.__bucket_motor_midpoint - self.__bucket_motor_speed * self.__is_digging)
                rospy.loginfo("\n\n" + RobotInterface.controller1_outstring + "\n\n" + RobotInterface.controller2_outstring)
        else:
            if RobotInterface.controller2_outstring_changed:
                RobotInterface.controller2_outstring_changed = False
                RobotInterface.controller2_outstring = "Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name) + "\nIs Digging: " + str(self.__is_digging) + "\nIs Conveying: " + str(self.__is_conveying) + "\nIs Sifting: " + str(self.__is_sifting) + "\nFront Actuators Inversion: " + str(self.__front_actuator_inversion) + "\nBack Actuators Inversion: " + str(self.__back_actuator_inversion) + "\nCurrent Digging Speed: " + str(self.__bucket_motor_midpoint - self.__bucket_motor_speed * self.__is_digging)
                rospy.loginfo("\n\n" + RobotInterface.controller1_outstring + "\n\n" + RobotInterface.controller2_outstring)

        #Build the message to send over UDP
        message = str(self.FRONT_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.FRONT_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"

        message += str(self.FRONT_LEFT_ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["LT"]] - 1) / -2) * self.__front_actuator_inversion) + "\n"
        message += str(self.FRONT_RIGHT_ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["LT"]] - 1) / -2) * self.__front_actuator_inversion) + "\n"
        message += str(self.BACK_LEFT_ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["RT"]] - 1) / -2) * self.__back_actuator_inversion) + "\n"
        message += str(self.BACK_RIGHT_ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["RT"]] - 1) / -2) * self.__back_actuator_inversion) + "\n"


        message += str(self.BUCKET_MOTOR) + "," + str(self.__bucket_motor_midpoint - self.__bucket_motor_speed * self.__is_digging) + "\n"
        message += str(self.CONVEYOR_MOTOR) + "," + str(self.__conveyor_motor_midpoint + self.__conveyor_motor_increment_value * self.__is_conveying * self.__conveyor_direction) + "\n"

        if RobotInterface.artificial_delay_enabled:
            delay = gauss(self.mu, self.sigma)
            sleep(delay)

        #rospy.loginfo("\nmessage:\n" + message)

        #self.set_point.sendMessage(message)


#Inherited class for the Sensor Tower
class DumperInterface(RobotInterface):

    def __init__(self, ip):
        RobotInterface.__init__(self, ip)
        self.__my_name = "Sensor Tower"
        self.__my_selector = RobotInterface.interface_count
        RobotInterface.interface_count += 1

        #Motor Indices on embedded side
        self.FRONT_LEFT_DRIVE = 0
        self.FRONT_RIGHT_DRIVE = 1
        self.BACK_RIGHT_DRIVE = 2
        self.BACK_LEFT_DRIVE = 3

        self.SENSOR_TOWER_BASE_MOTOR = 4
        self.SENSOR_TOWER_TOP_MOTOR = 5

        self.ACTUATOR = 6

        #Values for drive motor value calculation
        self.__drive_motor_midpoint = 511.0
        self.__drive_motor_increment_value = 171.0

        #Values for arm motor value calculations
        #self.__arm_motors_base = 0
        self.__base_arm_inversion = 1
        self.__top_arm_inversion = 1

        #Values for actuator value calculation
        self.__actuator_midpoint = 511.0
        self.__actuator_increment_value = 171.0

        self.__actuator_inversion = 1

        #self.__arm_motors_top = 0
        #self.__arm_motors_base_max = -140 * (math.pi/180)
        #self.__arm_motors_top_max = 100 * (math.pi/180)
        #self.__arm_reflection = 120 * (math.pi/180)
        self.__top_arm_toggle = rospy.Time.now()
        self.__base_arm_toggle = rospy.Time.now()
        self.__actuator_inversion_toggle_time = rospy.Time.now()
        self.__arm_motor_midpoint = 511.0
        self.__motor_increment_value = 171.0

    #Callback override for the Joy controller Topic
    def handleControllerCallback(self, controller_state, controller_id):

        #If we're not currently trying to control this robot just return       
        if controller_id == 1: 
            if self.__my_selector != RobotInterface.controller1_interface_select:
                return
        else:
            if self.__my_selector != RobotInterface.controller2_interface_select:
                return

        #Change currently selected robot
        if controller_state.buttons[self.buttons["BACK"]]:
            if controller_id == 1:
                duration = rospy.Time.now() - RobotInterface.controller1_interface_select_toggle_time

                if duration.to_sec() > 1:
                    RobotInterface.controller1_interface_select += 1
                        
                    RobotInterface.controller1_interface_select = RobotInterface.controller1_interface_select % (RobotInterface.interface_count)

                    if RobotInterface.controller1_interface_select == RobotInterface.controller2_interface_select:
                        RobotInterface.controller1_interface_select += 1

                    RobotInterface.controller1_interface_select = RobotInterface.controller1_interface_select % (RobotInterface.interface_count)

                    RobotInterface.controller1_interface_select_toggle_time = rospy.Time.now()

                    RobotInterface.controller1_outstring_changed = True

            else:
                duration = rospy.Time.now() - RobotInterface.controller2_interface_select_toggle_time

                if duration.to_sec() > 1:
                    RobotInterface.controller2_interface_select += 1

                    RobotInterface.controller2_interface_select = RobotInterface.controller2_interface_select % (RobotInterface.interface_count)

                    if RobotInterface.controller2_interface_select == RobotInterface.controller1_interface_select:
                        RobotInterface.controller2_interface_select += 1

                    RobotInterface.controller2_interface_select = RobotInterface.controller2_interface_select % (RobotInterface.interface_count)

                    RobotInterface.controller2_interface_select_toggle_time = rospy.Time.now()

                    RobotInterface.controller2_outstring_changed = True

            return


        #if controller_state.axes[self.axes["LT"]]==-1:
        #    self.__arm_motors_base+=-5*self.__base_arm_inversion * (math.pi/180)
        #    if self.__arm_motors_base > self.__arm_motors_base_max:
        #        self.__arm_motors_base=self.__arm_motors_base_max
        #    elif self.__arm_motors_base < 0:
        #        self.__arm_motors_base=0
        #    else:
        #        if (self.__arm_motors_base <= self.__arm_reflection and self.__base_arm_inversion==1) or (self.__base_arm_inversion==-1 and self.__arm_motors_base<self.__arm_reflection):
        #            self.__arm_motors_top+=5*self.__base_arm_inversion*-1 * (math.pi/180)
        #        else:
        #            self.__arm_motors_top+=5*self.__base_arm_inversion * (math.pi/180)
        #
        #if controller_state.axes[self.axes["RT"]]==-1:
        #    self.__arm_motors_top+=5*self.__top_arm_inversion * (math.pi/180)
        #    if self.__arm_motors_top > self.__arm_motors_top_max:
        #        self.__arm_motors_top=self.__arm_motors_top_max
        #    if self.__arm_motors_top<0:
        #        self.__arm_motors_top=0
        #
        if controller_state.buttons[self.buttons["RB"]]:
            duration = rospy.Time.now() - self.__base_arm_toggle

            if duration.to_sec() > 1:
                self.__base_arm_inversion= self.__base_arm_inversion*-1
                self.__base_arm_toggle=rospy.Time.now()

        if controller_state.buttons[self.buttons["LB"]]:
            duration = rospy.Time.now() - self.__actuator_inversion_toggle_time

            if duration.to_sec() > 1:
                #self.__top_arm_inversion= self.__top_arm_inversion*-1
                #self.__top_arm_toggle=rospy.Time.now()
                self.__actuator_inversion *= -1
                self.__actuator_inversion_toggle_time = rospy.Time.now()

                
        for button in controller_state.buttons:
            if button:
                if controller_id == 1:
                    RobotInterface.controller1_outstring_changed = True
                else:
                    RobotInterface.controller2_outstring_changed = True


        if controller_id == 1:
            if RobotInterface.controller1_outstring_changed:
                RobotInterface.controller1_outstring_changed = False
                RobotInterface.controller1_outstring = "Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name) + "\nTop Arm Inversion:" + str(self.__top_arm_inversion)+"\nBase Arm Inversion:" + str(self.__base_arm_inversion) + "\nActuator Inversion: " + str(self.__actuator_inversion)
                rospy.loginfo("\n\n" + RobotInterface.controller1_outstring + "\n\n" + RobotInterface.controller2_outstring)
        else:
            if RobotInterface.controller2_outstring_changed:
                RobotInterface.controller2_outstring_changed = False
                RobotInterface.controller1_outstring = "Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name) + "\nTop Arm Inversion:" + str(self.__top_arm_inversion)+"\nBase Arm Inversion:" + str(self.__base_arm_inversion) + "\nActuator Inversion: " + str(self.__actuator_inversion)
                rospy.loginfo("\n\n" + RobotInterface.controller1_outstring + "\n\n" + RobotInterface.controller2_outstring)

        #rospy.loginfo("Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name))

        #Build the message to send over UDP
        message = str(self.FRONT_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.FRONT_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"

        message += str(self.SENSOR_TOWER_BASE_MOTOR) + "," + str(self.__arm_motor_midpoint + ((controller_state.axes[self.axes["RT"]] - 1) / -2) * self.__motor_increment_value * self.__base_arm_inversion) + "\n"
        message += str(self.SENSOR_TOWER_TOP_MOTOR) + "," + str(511.0 + ((controller_state.axes[self.axes["LT"]] - 1) / -2) * 45.0 * self.__top_arm_inversion) + "\n"

        message += str(self.ACTUATOR) + "," + str(self.__actuator_midpoint + self.__actuator_increment_value * ((controller_state.axes[self.axes["LT"]] - 1) / -2) * self.__actuator_inversion) + "\n"

        if RobotInterface.artificial_delay_enabled:
            delay = gauss(self.mu, self.sigma)
            sleep(delay)

        #rospy.loginfo("\nmessage:\n" + message)

        self.set_point.sendMessage(message)


#Inherited class for the minibot
class TransporterInterface(RobotInterface):

    def __init__(self, ip):
        RobotInterface.__init__(self, ip)
        self.__my_name = "Mini Bot"
        self.__my_selector = RobotInterface.interface_count
        RobotInterface.interface_count += 1

        #Motor Indices on embedded side
        self.FRONT_LEFT_DRIVE = 0
        self.FRONT_RIGHT_DRIVE = 1
        self.BACK_RIGHT_DRIVE = 2
        self.BACK_LEFT_DRIVE = 3

        #Values for drive motor value calculation
        self.__drive_motor_midpoint = 0
        self.__drive_motor_increment_value = 2 * math.pi
        self.__drive_motor_max_fraction = 1.0

    #Callback override for the Joy controller Topic
    def handleControllerCallback(self, controller_state, controller_id):
        
        #If we're not currently trying to control this robot just return       
        if controller_id == 1: 
            if self.__my_selector != RobotInterface.controller1_interface_select:
                return
        else:
            if self.__my_selector != RobotInterface.controller2_interface_select:
                return

        #Change currently selected robot
        if controller_state.buttons[self.buttons["BACK"]]:
            if controller_id == 1:
                duration = rospy.Time.now() - RobotInterface.controller1_interface_select_toggle_time

                if duration.to_sec() > 1:
                    RobotInterface.controller1_interface_select += 1
                        
                    RobotInterface.controller1_interface_select = RobotInterface.controller1_interface_select % (RobotInterface.interface_count)

                    if RobotInterface.controller1_interface_select == RobotInterface.controller2_interface_select:
                        RobotInterface.controller1_interface_select += 1

                    RobotInterface.controller1_interface_select = RobotInterface.controller1_interface_select % (RobotInterface.interface_count)

                    RobotInterface.controller1_interface_select_toggle_time = rospy.Time.now()

                    RobotInterface.controller1_outstring_changed = True

            else:
                duration = rospy.Time.now() - RobotInterface.controller2_interface_select_toggle_time

                if duration.to_sec() > 1:
                    RobotInterface.controller2_interface_select += 1

                    RobotInterface.controller2_interface_select = RobotInterface.controller2_interface_select % (RobotInterface.interface_count)

                    if RobotInterface.controller2_interface_select == RobotInterface.controller1_interface_select:
                        RobotInterface.controller2_interface_select += 1

                    RobotInterface.controller2_interface_select = RobotInterface.controller2_interface_select % (RobotInterface.interface_count)

                    RobotInterface.controller2_interface_select_toggle_time = rospy.Time.now()

                    RobotInterface.controller2_outstring_changed = True

            return
                
        if controller_state.buttons[self.buttons["B"]]:
            if self.__drive_motor_max_fraction == 1.0:
                self.__drive_motor_max_fraction = 0.5
            else:
                self.__drive_motor_max_fraction = 1.0


        for button in controller_state.buttons:
            if button:
                if controller_id == 1:
                    RobotInterface.controller1_outstring_changed = True
                else:
                    RobotInterface.controller2_outstring_changed = True

        if controller_id == 1:
            if RobotInterface.controller1_outstring_changed:
                RobotInterface.controller1_outstring_changed = False
                RobotInterface.controller1_outstring = "Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name) + "\nCurrent Max Drive Fraction: " + str(self.__drive_motor_max_fraction)
                rospy.loginfo("\n\n" + RobotInterface.controller1_outstring + "\n\n" + RobotInterface.controller2_outstring)
        else:
            if RobotInterface.controller2_outstring_changed:
                RobotInterface.controller2_outstring_changed = False
                RobotInterface.controller1_outstring = "Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name) + "\nCurrent Max Drive Fraction: " + str(self.__drive_motor_max_fraction)
                rospy.loginfo("\n\n" + RobotInterface.controller1_outstring + "\n\n" + RobotInterface.controller2_outstring)

        #rospy.loginfo("Controller (" + str(controller_id) + ") Selected Robot: " + str(self.__my_name))

        #Build the message to send over UDP
        message = str(self.FRONT_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_LEFT_DRIVE) + "," + str(self.__drive_motor_midpoint - self.__drive_motor_increment_value * controller_state.axes[self.axes["LS_UP_DOWN"]]) + "\n"
        message += str(self.FRONT_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"
        message += str(self.BACK_RIGHT_DRIVE) + "," + str(self.__drive_motor_midpoint + self.__drive_motor_increment_value * controller_state.axes[self.axes["RS_UP_DOWN"]]) + "\n"

        if RobotInterface.artificial_delay_enabled:
            delay = gauss(self.mu, self.sigma)
            sleep(delay)

        #rospy.loginfo("\nmessage:\n" + message)

        #self.set_point.sendMessage(message)


def main():

    #RobotInterface.artificial_delay_enabled = True

    transporter = TransporterInterface("192.168.0.32")
    dumper = DumperInterface("192.168.0.100")
    digger = DiggerInterface("192.168.0.101")

    rospy.spin()

if __name__ == '__main__':
    main()