"""
Functions for listening to events in the rover environment.

NOTE:
- angles are in degrees
- distances are in meters
- speeds are in meters/second

"""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import numpy as np

from constants import TO_DEG


def velocity_exceeded(Rover, max_vel=2.0):
    """
    Check if velocity is under max_vel.

    Keyword arguments:
    max_vel -- maximum velocity in meters/second
    """
    return Rover.vel < max_vel


def front_path_clear(Rover, safe_pixs=500):
    """
    Check if sufficient room ahead.

    Keyword arguments:
    safe_pixs -- minimum number of pixels in front to deem front path clear
    """
    nav_pixs_front = len(Rover.nav_angles)
    return nav_pixs_front >= safe_pixs


def left_path_clear(Rover, safe_pixs=1500):
    """
    Check if sufficient room on left.

    Keyword arguments:
    safe_pixs -- minimum number of pixels on left to deem left path clear
    """
    nav_pixs_left = len(Rover.nav_angles_left)
    return nav_pixs_left >= safe_pixs


def pointed_at_nav(Rover, angle_limit=17):
    """
    Check if rover is pointed within some range of average nav heading.

    Keyword arguments:
    angle_limit -- angle range limit for nav angles (degrees)
    """
    nav_heading = np.mean(Rover.nav_angles)*TO_DEG
    return -angle_limit <= nav_heading <= angle_limit


def pointed_along_wall(Rover, safe_pixs=500, wall_angle_bias=-10):
    """
    Check if rover is pointing along left wall at some safe distance.

    Keyword arguments:
    safe_pixs --  minimum number of pixels to keep from wall
    wall_angle_bias -- to bias rover heading for pointing along wall (degrees)
    """
    nav_pixs_left = len(Rover.nav_angles_left)
    nav_heading_left = np.mean(Rover.nav_angles_left)*TO_DEG + wall_angle_bias

    return (nav_pixs_left >= safe_pixs
            and nav_heading_left > 0)


def deviated_from_wall(Rover, max_angle_wall=25):
    """
    Check if rover has deviated from left wall beyond specified angle limit.

    Keyword arguments:
    max_angle_wall --  maximum allowed angle from left wall (degrees)
    """
    nav_heading_left = np.mean(Rover.nav_angles_left)*TO_DEG
    return nav_heading_left > max_angle_wall


def at_front_obstacle(Rover, safe_pixs=600):
    """
    Check if rover is up against some obstacle on the front.

    Keyword arguments:
    safe_pixs -- minimum number of pixels to keep from front obstacles
    """
    nav_pixs_front = len(Rover.nav_angles)
    return nav_pixs_front < safe_pixs


def at_left_obstacle(Rover, safe_pixs=50):
    """
    Check if obstacle is to the left of rover.

    Keyword arguments:
    safe_pixs -- minimum number of pixels to keep from left obstacles
    """
    nav_pixs_left = len(Rover.nav_angles_left)
    return nav_pixs_left < safe_pixs


def sample_on_left(Rover, min_left_angle=0.0):
    """Check if a sample is spotted on the left.

    Keyword arguments:
    min_left_angle -- only rocks to the left/above this are considered
    """
    rock_heading = np.mean(Rover.rock_angles)*TO_DEG
    return rock_heading >= min_left_angle


def sample_right_close(Rover, rock_dist_limit=75, max_right_angle=17):
    """
    Check if a nearby sample is spotted on the right.

    Keyword arguments:
    rock_dist_limit -- only rocks below this limit are considered
    max_right_angle -- only rocks to left/above of this are considered
    """
    rock_heading = np.mean(Rover.rock_angles)*TO_DEG
    rock_distance = np.mean(Rover.rock_dists)

    return (rock_heading > -max_right_angle
            and rock_distance < rock_dist_limit)


def sample_in_view(Rover):
    """Check if rock sample still in view."""
    rock_pixs = len(Rover.rock_angles)
    return rock_pixs >= 1


def pointed_at_sample(Rover, angle_limit=17):
    """
    Check if rover is pointed within some range of average rock heading.

    Keyword arguments:
    angle_limit -- angle range limit for rock angles (radians)
    """
    rock_heading = np.mean(Rover.rock_angles)*TO_DEG
    return -angle_limit < rock_heading < angle_limit


def can_pickup_sample(Rover):
    """Check if Rover ready to pickup sample."""
    return Rover.near_sample and Rover.vel <= 0.1


def completed_mission(Rover, min_samples=6, min_mapped=95, max_time=700):
    """Check if rover has completed mission criteria."""
    return (Rover.samples_found >= min_samples
            and Rover.perc_mapped >= min_mapped
            ) or Rover.total_time >= max_time


def reached_home(Rover, max_dist=5):
    """Check if rover has reached home after completing mission."""
    return Rover.going_home and Rover.distance_to_home < max_dist
