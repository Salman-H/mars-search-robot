"""
Module for rover decision-handling.

Used to build a decision tree for determining throttle, brake and
steer commands based on the output of the perception_step() function
in the perception module.

NOTE:
time -- seconds
distance -- meters
velocity -- meters/second
angle, heading -- degrees
yaw, pitch, roll -- degrees

"""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import time

import numpy as np

import events
import states
import handlers


class DecisionSupervisor():
    """Handle events and switch between states."""

    def __init__(self):
        """Initialize a DecisionSupervisor instance."""
        # Define the set of state identifiers
        self.state = {
            0: states.FindWall(),
            1: states.FollowWall(),
            2: states.TurnToWall(),
            3: states.AvoidWall(),
            4: states.AvoidObstacles(),
            5: states.GoToSample(),
            6: states.Stop(),
            7: states.InitiatePickup(),
            8: states.WaitForPickupInitiate(),
            9: states.WaitForPickupFinish(),
            10: states.GetUnstuck(),
            11: states.ReturnHome(),
            12: states.Park()
        }
        # Define the set of events
        self.event = {
            'velocity_exceeded': events.velocity_exceeded,
            'front_path_clear': events.front_path_clear,
            'left_path_clear': events.left_path_clear,
            'pointed_at_nav': events.pointed_at_nav,
            'pointed_along_wall': events.pointed_along_wall,
            'deviated_from_wall': events.deviated_from_wall,
            'at_front_obstacle': events.at_front_obstacle,
            'at_left_obstacle': events.at_left_obstacle,
            'sample_on_left': events.sample_on_left,
            'sample_right_close': events.sample_right_close,
            'sample_in_view': events.sample_in_view,
            'pointed_at_sample': events.pointed_at_sample,
            'can_pickup_sample': events.can_pickup_sample,
            'completed_mission': events.completed_mission,
            'reached_home': events.reached_home
        }
        # Default state
        self.curr_state = self.state[0]  # FindWall
        self.starttime = 0.0  # for timer

    def is_event(self, Rover, name):
        """Check if given event has occurred."""
        func = self.event.get(name)
        return func(Rover)

    def either_events(self, Rover, name1, name2):
        """Check if either events have occurred."""
        func1 = self.event.get(name1)
        func2 = self.event.get(name2)
        return func1(Rover) or func2(Rover)

    def both_events(self, Rover, name1, name2):
        """Check if both events have occurred."""
        func1 = self.event.get(name1)
        func2 = self.event.get(name2)
        return func1(Rover) and func2(Rover)

    def is_state(self, name):
        """Check if handler is in given state."""
        return name is self.curr_state

    def switch_to_state(self, Rover, name):
        """Update current state to the next state."""
        name.execute(Rover)
        self.curr_state = name

    def is_stuck_for(self, Rover, stucktime):
        """Check if rover is stuck for stucktime."""
        exceeded_stucktime = False
        # If not moving then check since when
        if Rover.vel < 0.1:
            if not Rover.timer_on:
                self.starttime = time.time()  # start timer
                Rover.stuck_heading = Rover.yaw
                Rover.timer_on = True
            else:
                endtime = time.time()
                exceeded_stucktime = (endtime - self.starttime) > stucktime
        else:  # if started to move then switch OFF/Reset timer
            Rover.timer_on = False
            Rover.stuck_heading = 0.0
        return exceeded_stucktime

    def execute(self, Rover):
        """Select and call the handler for the current state."""
        # Ensure Rover telemetry data is coming in
        if Rover.nav_angles is not None:
            # State identifiers and corresponding handlers
            select = {
                self.state[0]: handlers.finding_wall,
                self.state[1]: handlers.following_wall,
                self.state[2]: handlers.turning_to_wall,
                self.state[3]: handlers.avoiding_wall,
                self.state[4]: handlers.avoiding_obstacles,
                self.state[5]: handlers.going_to_sample,
                self.state[6]: handlers.stopped_at_sample,
                self.state[7]: handlers.initiating_pickup,
                self.state[8]: handlers.waiting_pickup_initiate,
                self.state[9]: handlers.waiting_pickup_finish,
                self.state[10]: handlers.getting_unstuck,
                self.state[11]: handlers.returning_home,
                self.state[12]: handlers.parking
            }
            # Select and call the handler function for the current state
            func = select.get(self.curr_state, lambda: "nothing")
            func(self, Rover)
        return Rover
