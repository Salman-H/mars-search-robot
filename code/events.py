"""Module for rover events."""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import time

import numpy as np


def velocity_exceeded(Rover):
    """Check if velocity is under max_vel."""
    return Rover.vel < Rover.max_vel


def front_path_clear(Rover):
    """Check if sufficient room ahead."""
    return len(Rover.nav_angles) >= Rover.go_forward


def left_path_clear(Rover):
    """Check if sufficient room on left."""
    return len(Rover.nav_angles_left) >= 1500


def pointed_at_nav(Rover):
    """Check if rover pointed at +/- some nav heading."""
    return (np.mean(Rover.nav_angles) <= 0.3 and
            np.mean(Rover.nav_angles) >= -0.3)


def pointed_along_wall(Rover):
    """Check if rover has sufficient nav pixels along left wall."""
    go_forward_wall = 500
    return (len(Rover.nav_angles_left) > go_forward_wall and
            np.mean(Rover.nav_angles_left*180 / np.pi)-10 > 0)


def deviated_from_wall(Rover):
    """Check if rover has deviated from wall beyond specified limit."""
    wall_angle_limit = 25
    return np.mean(Rover.nav_angles_left*180/np.pi) > wall_angle_limit


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


def is_stuck(self, max_stucktime, Rover):
    """Check if rover is stuck for max_stucktime."""
    starttime = 0.0
    exceeded_stucktime = False

    if Rover.vel < 0.1:  # If not moving

        if not Rover.timer_on:  # If timer OFF then start timer 
            starttime = time.time()
            Rover.stuck_heading = Rover.yaw
            Rover.timer_on = True
        
        else:  # If timer already ON then check if time exceeded
            endtime = time.time()
            stucktime = endtime - starttime
            exceeded_stucktime = stucktime > max_stucktime

    else:  # Otherwise if started to move then reset timer
        Rover.timer_on = False
        Rover.stuck_heading = 0.0

    return exceeded_stucktime


def completed_mission(Rover):
    """Check if rover has completed mission criteria."""
    return ((Rover.samples_found >= 6 and Rover.perc_mapped >= 95.0) or
            Rover.total_time >= 700)


def reached_home(Rover):
    """Check if rover has reached home after completing mission."""
    return Rover.returning_home and Rover.distance_to_home < 5
