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