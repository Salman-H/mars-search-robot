"""
Module for handling switching from states.

NOTE:
time -- seconds
distance -- meters
velocity -- meters/second
angle, heading -- degrees
yaw, pitch, roll -- degrees

"""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


def finding_wall(Decider, Rover):
    """Handle switching from FindWall state."""
    if 45 < Rover.yaw < 65:
        Decider.switch_to_state(Rover, Decider.state[1])  # FollowWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def following_wall(Decider, Rover):
    """Handle switching from FollowWall state."""
    # Time in seconds allowed to remain stuck in this state
    stucktime = 2.0
    if Decider.both_events(Rover, 'deviated_from_wall', 'left_path_clear'):
        Decider.switch_to_state(Rover, Decider.state[2])  # TurnToWall

    elif Decider.is_event(Rover, 'at_left_obstacle'):
        Decider.switch_to_state(Rover, Decider.state[3])  # AvoidWall

    elif Decider.either_events(Rover, 'sample_on_left', 'sample_right_close'):
        Decider.switch_to_state(Rover, Decider.state[5])  # GoToSample

    elif Decider.is_event(Rover, 'completed_mission'):
        Rover.going_home = True
        Decider.switch_to_state(Rover, Decider.state[11])  # ReturnHome

    elif Decider.is_stuck_for(Rover, stucktime):
        Decider.switch_to_state(Rover, Decider.state[10])  # GetUnstuck
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def turning_to_wall(Decider, Rover):
    """Handle switching from TurnToWall state."""
    if Decider.is_event(Rover, 'pointed_along_wall'):
        Decider.switch_to_state(Rover, Decider.state[1])  # FollowWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def avoiding_wall(Decider, Rover):
    """Handle switching from AvoidWall state."""
    if Decider.is_event(Rover, 'pointed_along_wall'):
        Decider.switch_to_state(Rover, Decider.state[1])  # FollowWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def avoiding_obstacles(Decider, Rover):
    """Handle switching from AvoidObstacles state."""
    if Decider.both_events(Rover, 'front_path_clear', 'pointed_at_nav'):
        Decider.switch_to_state(Rover, Decider.state[11])  # ReturnHome
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def going_to_sample(Decider, Rover):
    """Handle switching from GoToSample state."""
    # Time in seconds allowed to remain stuck in this state
    stucktime = 2.5
    if Decider.is_event(Rover, 'sample_in_view'):
        if Rover.near_sample:
            Rover.timer_on = False
            Decider.switch_to_state(Rover, Decider.state[6])  # Stop

        elif Decider.is_stuck_for(Rover, stucktime):
            Rover.timer_on = False
            Decider.switch_to_state(Rover, Decider.state[10])  # GetUnstuck
        else:
            Decider.switch_to_state(Rover, Decider.curr_state)


def stopped_at_sample(Decider, Rover):
    """Handle switching from Stop state."""
    if Decider.is_event(Rover, 'can_pickup_sample'):
        Decider.switch_to_state(Rover, Decider.state[7])  # InitiatePickup
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def initiating_pickup(Decider, Rover):
    """Handle switching from InitiatePickup state."""
    Decider.switch_to_state(Rover, Decider.state[8])  # WaitForPickupInitiate


def waiting_pickup_initiate(Decider, Rover):
    """Handle switching from WaitForPickupInitiate state."""
    if Rover.picking_up == 1:
        Decider.switch_to_state(Rover, Decider.state[9])  # WaitForPickupFinish
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def waiting_pickup_finish(Decider, Rover):
    """Handle switching from WaitForPickupFinish state."""
    if Rover.picking_up == 0:
        Decider.switch_to_state(Rover, Decider.state[3])  # AvoidWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def getting_unstuck(Decider, Rover):
    """Handle switching from GetUnstuck state."""
    # If reached sufficient velocity or stuck while in GetUnstuck state
    # then get out of GetUnstuck state
    stucktime = 2.0
    if Rover.vel >= 1.0 or Decider.is_stuck_for(Rover, stucktime):
        if Rover.going_home:
            Decider.switch_to_state(Rover, Decider.state[11])  # ReturnHome
        else:
            Decider.switch_to_state(Rover, Decider.state[1])  # FollowWall
    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def returning_home(Decider, Rover):
    """Handle switching from ReturnHome state."""
    # Time in seconds allowed to remain stuck in this state
    stucktime = 2.0
    if Decider.is_event(Rover, 'at_front_obstacle'):
        Decider.switch_to_state(Rover, Decider.state[11])  # ReturnHome

    elif Decider.is_event(Rover, 'reached_home'):
        Decider.switch_to_state(Rover, Decider.state[12])  # Park

    elif Decider.is_stuck_for(Rover, stucktime):
        Decider.switch_to_state(Rover, Decider.state[10])  # GetUnstuck

    else:
        Decider.switch_to_state(Rover, Decider.curr_state)


def parking(Decider, Rover):
    """Handle switching from Park state."""
    Decider.switch_to_state(Rover, Decider.state[12])  # Remain in Park
