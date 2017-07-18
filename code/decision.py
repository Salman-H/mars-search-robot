"""
Module for rover decision-handling.

Used to build a decision tree for determining throttle, brake and
steer commands based on the output of the perception_step() function
in the perception module.

"""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import time

import numpy as np

import states
import events
