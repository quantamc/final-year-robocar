#!/usr/bin/python

import rospy
from i2cpwm_board.msg import Servo, ServoArray
from geometry_msgs.msg import Twist
import time

class ServoConvert():
    """
    Class for handling the servos fro the 12c control 12cpem_board
    """

    def __init__(self, id=1, centre_value=360, range=90, direction=1):
        self.value          = 0.0
        self.centre_value   = centre_value
        self.id             = id

        self._centre        = centre_value
        self._range         = range
        self._half_range    = 0.5*range
        self._dir           = direction


        #-- convert the range in [-1, 1]
        self._sf                = 1.0/self._half_range


    def get_value_out(self, value_in):
        """  
        given an input reference in [-1, 1], it converts the it in the 
        actual servo range
        """
        self.value          = value_in
        self.value_out      = int(self._dir*value_in*self._half_range + self._centre)
        return(self.value_out)


class TsLowLevelCtrl():
    """
    Low level control for the transi in ROS
    """
    def __init__(self):
        rospy.loginfo("Setting up the node...")

        rospy.init_node("ts_llc")

        #--- Create the actuator dictionary
        self.actuators = {}
        self.actuators['throttle'] = ServoConvert(id=1, centre_value=0)
        self.actuators['steering'] = ServoConvert(id=2, direction=1)#-positive left

        #--- Create the servo array publisher
        #-- Create the message
        self._servo_msg     = ServoArray()
        for i in range(2):
            self._servo_msg.servos.append(Servo())

        self.ros_pub_servo_array = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher correctly initialized")

        #--- Create the subscriber to the /cmd-vel topic
        self.ros_sub_twist = rospy.Subscriber("/cmd_vel", Twist, self.set_actuator_from_cmdvel)
        rospy.loginfo("> subscriber correctly initialized")

        #--- save the last time we got a reference. Stop the vehicle if no commands given
        self._last_time_cmd_rcv     = time.time()
        self._timeout_s             = 5 #-- stop after 5 seconds 

        rospy.loginfo("Initialization complete")

    def set_actuator_from_cmdvel(self, message):
        """
        Get a Twist message from cmd_vel, assuming max inmput is 1
        """
        #-- Save the time
        self._last_time_cmd_rcv = time.time()

        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(message.linear.x)
        self.actuators['steering'].get_value_out(message.angular.z)
        rospy.loginfo("Got a command v = %2.1f  s=%2.1f"%(message.linear.x, message.angular.z))
        self.send_servo_msg()

    def set_actuators_idle(self):
        self.actuators['throttle'].get_value_out(0) #- positive fwd
        self.actuators['steering'].get_value_out(0) #-positve right
        rospy.loginfo('Setting Actuators to idle')

        #-- Puplish the message using a function
        self.send_servo_msg()

    def send_servo_msg(self):
        """
        Publish the current actuators' value to the i2cpwm_board
        Servos = array of the Servo. Each Servo has an id and a value to fill

        """
        for actuator_name , servo_obj in self.actuators.iteritems():
            self._servo_msg.servos[servo_obj.id-1].servo    = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value    = servo_obj.value_out
            rospy.loginfo("sending to %s command %d"%(actuator_name, servo_obj.value_out))

        self.ros_pub_servo_array.publish(self._servo_msg)

    @property
    def is_controller_connected(self):
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)


    def run(self):

        #-- Set the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            print self._last_time_cmd_rcv, self.is_controller_connected

            if not self.is_controller_connected:
                self.set_actuators_idle()

            rate.sleep()

"""
Execute the main file
"""
if __name__ == "__main__":
    ts_llc = TsLowLevelCtrl()
    ts_llc.run()

        