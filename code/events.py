"""
Module for rover events.

"""
__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import numpy as np


def velocity_exceeded(Rover):
    """Check if velocity is under max_vel."""
    return Rover.vel < Rover.max_vel


def front_path_clear(Rover):
    """Check if sufficient room ahead."""
    return len(Rover.nav_angles) >= Rover.go_forward


def can_pickup_sample(Rover):
    """Check if Rover ready to pickup sample."""
    return Rover.near_sample and Rover.vel <= 0.1
