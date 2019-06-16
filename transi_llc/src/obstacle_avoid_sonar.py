#!/usr/bin/python

"""
Sonar array composed of 3 sonars (left, centre, right)

Subcribes to the three topics 0 (left) 1(centre) 2 (right)

calculates a correction to the cmd_vel that is:
    multiplicative for x
    additive for angle

"""

import math, time
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

DIST_STEER_ENGAGE   = 1.2
DIST_BREAK          = 0.4

DIST_LATE_ENGAGE    = 0.4

K_FRONT_DIST_TO_SPEED   = 1.0
K_LAT_DIST_TO_STEER     = 2.0

TIME_KEEP_STEERING      = 1.5

def saturate(value,min, max):
    if value <= min: return(min)
    elif value >= max: return(max)
    else: return(value)


class ObstAvoid():
    def __init__(self):

        self.range_centre   = 3
        self.range_left     = 3
        self.range_right    = 3

        self.sub_centre = rospy.Subscriber("transi/sonar/1", Range, self.update_range)
        self.sub_left = rospy.Subscriber("transi/sonar/0", Range, self.update_range)
        self.sub_right = rospy.Subscriber("trabsi/sonar/2", Range, self.update_range)
        rospy.loginfo('Subscribers set')

        self.pub_twist = rospy.Publisher("tranis/control/cmd_vel", Twist, queue_size=5)
        rospy.loginfo("Publisher set")

        self._message =Twist()

        self._time_steer    = 0
        self._steer_sign_prev   = 0

    def update_range(self, message):
        angle = message.field_of_view

        if abs(angle) < 0.1:
            self.range_centre = message.range

        elif angle > 0:
            self.range_right = message.range

        elif angle < 0:
            self.range_left = message.range

        # rospy.loginfo("Sonar array: %.1f  %.1f  %.1f"%(self.range_left, self.range_center, self.range_right))

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """


        break_action = 1.0
        steer_action = 0.0

        #--- Get the minimum distance
        range = min([self.range_centre, slef.range
        ])