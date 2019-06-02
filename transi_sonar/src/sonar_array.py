#!/usr/bin/python

"""
Sonar array composed of 3 sonars (left, centre, right)
"""


from sonar import Sonar'
import math
import rospy
from sensor_msgs.msg import Range

"""
Sensor_msgs/Range

unit8  ULTRASOUND=0
unit8 INFRARED=1
std_msgs/Header header
    unit32 seq
    time stamp
    string frame_id
unit8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range

"""

#-------- PARAMETERS
NUM_SONAR           = 3
#-- CENTRE
SONAR_CENTRE_GPIO_TRIGGER      = 5
SONAR_CENTRE_GPIO_ECHO         = 6

#-- RIGHT
SONAR_RIGHT_GPIO_TRIGGER        = 27
SONAR_RIGHT_GPIO_ECHO           = 22

#-- LEFT
SONAR_LEFT_GPIO_TRIGGER         = 4
SONAR_LEFT_GPIO_ECHO            = 17

class SonarArray():
    def __init__(self,
            num_sonar,
            gpio_trigger_list,  #--list of all the trigger pins, starting from left
            gpio_echo_list,     #--list of all the echo pins, satrting from left
            range_min,         #- [m]
            range_max,         #- [m]
            angle_min_deg,      #[deg]
            angle_max_deg
            ):

        self.sonar_array    = []
        self.pub_array      = []
        self.num_sonar      = num_sonar

        delta_angle_deg =  (angle_max_deg-angle_min_deg)/float(num_sonar-1)

        rospy.loginfo("Initializing the arays")
        #--- Create an array and expand the object with its angle
        for i in range(num_sonar):
            sonar       = Sonar(gpio_trigger_list[i], gpio_echo_list[i], range_min=range_min*100 )