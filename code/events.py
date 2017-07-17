"""Module for rover events."""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import numpy as np


def velocity_exceeded(Rover):
    """Check if velocity is under max_vel."""
    return Rover.vel < Rover.max_vel


def front_path_clear(Rover):
    """Check if sufficient room ahead."""
    return len(Rover.nav_angles) >= Rover.go_forward


def at_front_obstacle(Rover):
    """Check if rover is up against some obstacle."""
    front_stop_forward = 600
    return len(Rover.nav_angles) < front_stop_forward


def at_left_obstacle(Rover):
    """Check if obstacle is left of rover."""
    return len(Rover.nav_angles_left) < Rover.stop_forward


def sample_on_left(Rover):
    """Check if sample spotted on the left."""
    return len(Rover.rock_angles) >= 1 and np.mean(Rover.rock_angles) >= 0


def sample_right_close(Rover):
    """Check if sample on the right and close by."""
    return np.mean(Rover.rock_angles) > -0.3 and np.mean(Rover.rock_dists) < 75


def sample_in_view(Rover):
    """Check if sample still in view."""
    return len(Rover.rock_angles) >= 1


def pointed_at_sample(Rover):
    """Check if rover pointed at +/- some sample heading."""
    return (len(Rover.rock_angles) >= 1 and
               (np.mean(Rover.rock_angles) < 0.3 and
                np.mean(Rover.rock_angles) > -0.3))


def can_pickup_sample(Rover):
    """Check if Rover ready to pickup sample."""
    return Rover.near_sample and Rover.vel <= 0.1
