#/usr/bin/python

"""
Class for low level control of car, requires ros-12cpwmboard installed

Listens to /tscar/control/cmd for corrective actions and avoid obstacles
"""

import rospy
import i2cpwm_board.msg import Servo, ServoArray
from geometry_msg.msg import Twist
import time

class ServoConvert():
    def __init__(self, id=1, centre_value=333, range=90, direction=1):
        self.value          = 0.0
        self.value_out      = centre_value
        self._centre        = centre_value
        self._range         = range
        self._half_range    = 0.5*range
        self._dir           = direction
        self.id             = id

        #------ Convert its range in [-1, 1]
        self._sf            = 1.0/self._half_range

    def get_value_out(self, value_in):
        #---- value is in [-1, 1]
        self.value          = value_in
        self.value_out      = int(self._dir*value_in*self._half_range + self._centre)
        # print self.id, self.value_out
        return(self.value_out)

def saturate(value, min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)


class TSLowLevelCtrl():
    def __init__(self):
        rospy.loginfo("Setting up the Node...")

        #------ Initialize the node
        rospy.init_node('ts_llc')

        self.actuators = {}
        self.actuators['throttle']  = ServoConvert(id=1)
        self.actuators['steering']  = ServoConvert(id=2, centre_value=328,direction=1) #-- positive left
        rospy.loginfo("> Actuators correctly initialized")

        self._servo_msg         = ServoArray()
        for i in range(2): self._servo_msg.servos.append(Servo())

        #------- Create the servo array publisher
        self.ros_pub_servo_array    = rospy.Publisher("/servos_absolute", ServoArray, queue_size=1)
        rospy.loginfo("> Publisher correctly initialized")

        #------- create a debug publisher for resulting cmd_vel
        self.ros_pub_debug_command  = rospy.Publisher("/tscar/control/cmd_vel", Twist, queue_size=1)
        rospy.loginfo("> Subscriber correctly initialized")

        #------- create the Subcriber to Twist commands
        self.ros_sub_twist          = rospy.Subscriber("/cmd_vel", Twist, self.update_messages_from_command)
        rospy.loginfo("> Subcriber correctly initalized")

        self.throttle_cmd       = 0.
        self.throttle_avoid     = 1.
        self.steer_cmd          = 0.
        self.steer_avoid        = 0. 

        self._debug_command_msg = Twist()

        #------- Get the last time we got a command
        self._last_time_cmd_rcv     = time.time()
        self._last_time_avoid_rcv   = time.time()
        self._timeout_s             = 5

        rospy.loginfo("initialization complete")

    def update_messages_from_command(self, message):
        self._last_time_cmd_rcv     = time.time()
        self.throttle_cmd           = message.linear.x
        self.steer_cmd              = message.angular.z

    def compose_command_velocity(self):
        self.throttle       = saturate(self.throttle_cmd*self.throttle_avoid, -1, 1)

        #-- Add steering only if positive throttle
        if self.throttle_cmd > 0:
            self.steer      = saturate(self.steer_cmd + self.steer_avoid, -1,1)
        else:
            self.steer =    = steer_cmd

        self._debug_command_msg.linear.x    = self.throttle
        self._debug_command_msg.angular.z   = self.steer

        self.set_actuators_from_cmdvel(self, throttle, self.steer)


    def set_actuators_from_cmdvel(self, throttle, steering):
        """
        Get a message from cmd_vel, assuming a maximum input of 1
        """
        #-- Convert vel into servo values
        self.actuators['throttle'].get_value_out(throttle)
        self.actuators['steering'].get_value_out(steering)
        rospy.loginfo("Got a command v = %2.1f  s=%2.1f"%(throttle, steering))
        self.send_servo_msg()

    def set_actuators_idle(self):
        #-- Convert vel into servo values
        self.throttle_cmd   = 0.
        self.steer_cmd      = 0.
    
    def reset_avoid(self):
        self.throttle_avoid = 1.
        self.steer_avoid    = 0.


    def send_servo_msg(self):
        for actuator_name, servo_obj in self.actuators.iteritems():
            self._servo_msg.servos[servo_obj.id-1].servo = servo_obj.id
            self._servo_msg.servos[servo_obj.id-1].value = servo_obj.value_out
            rospy.loginfo("Sendid to %s command %d"%(actuator_name, servo))

        self.ros_pub_servo_array.publisher(self._servo_msg)

    @property
    def is_controller_connected(self):
        return(time.time() - self._last_time_cmd_rcv < self._timeout_s)

    @property
    def is_avoid_connected(self):
        return(time.time() - self._last_time_avoid_rcv < self._timeout_s)

    def run(self):

        #--- se the control rate
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.compose_command_velocity()

            if not self.is_controller_connected:
                self.set_actuators_idle()

            
            if not self.is_avoid_connected:
                self.reset_avoid()

            

            rate.sleep()

if __name__ == "__main__":
    ts_llc  = TSLowLevelCtrl()
    ts_llc.run()
