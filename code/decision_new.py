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


class DecisionHandler():
    """Handle events and switch between states."""

    def __init__(self):
        """Initialize a DecisionHandler instance."""
        # define the set of states
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
        # define the set of events
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
        self.curr_state = self.state[0]  # default state

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
        # NOTE: Timer is switched ON whenever velocity drops below 0.1 m/s,
        #       Timer is switched OFF/Reset whenever stucktime is exceeded
        starttime = 0.0
        exceeded_stucktime = False
        if Rover.vel < 0.1:  # If not moving then check since when
            if not Rover.timer_on:  # If timer OFF then start timer
                starttime = time.time()
                Rover.stuck_heading = Rover.yaw
                Rover.timer_on = True
            else:  # If timer already ON then check if stucktime exceeded
                endtime = time.time()
                exceeded_stucktime = (endtime - starttime) > stucktime
                if exceeded_stucktime:
                    Rover.timer_on = False
        else:  # Otherwise if started to move then switch OFF/Reset timer
            Rover.timer_on = False
            Rover.stuck_heading = 0.0
        return exceeded_stucktime

    def finding_wall(self, Rover):
        """Handle switching from FindWall state."""
        if Rover.yaw > 45 and Rover.yaw < 65:
            self.switch_to_state(Rover, self.state[1])  # FollowWall
        else:
            self.switch_to_state(Rover, self.curr_state)

    def following_wall(self, Rover):
        """Handle switching from FollowWall state."""
        # time in seconds allowed to remain stuck in this state
        stucktime = 2.0
        if self.both_events(Rover, 'deviated_from_wall', 'left_path_clear'):
            self.switch_to_state(Rover, self.state[2])  # TurnToWall

        elif self.is_event(Rover, 'at_left_obstacle'):
            self.switch_to_state(Rover, self.state[3])  # AvoidWall

        elif self.either_events(Rover, 'sample_on_left', 'sample_right_close'):
            self.switch_to_state(Rover, self.state[5])  # GoToSample

        elif self.is_stuck_for(Rover, stucktime):
            self.switch_to_state(Rover, self.state[10])  # GetUnstuck
        else:
            self.switch_to_state(Rover, self.curr_state)

    def avoiding_wall(self, Rover):
        """Handle switching from AvoidWall state."""
        if self.is_event(Rover, 'pointed_along_wall'):
            self.switch_to_state(Rover, self.state[1])  # FollowWall
        else:
            self.switch_to_state(Rover, self.curr_state)

    def going_to_sample(self, Rover):
        """Handle switching from GoToSample state."""
        # time in seconds allowed to remain stuck in this state
        stucktime = 4.0
        if self.is_event(Rover, 'sample_in_view'):
            if Rover.near_sample:
                self.switch_to_state(Rover, self.state[6])  # Stop
            elif self.is_stuck_for(Rover, stucktime):
                self.switch_to_state(Rover, self.state[10])  # GetUnstuck
            else:
                self.switch_to_state(Rover, self.curr_state)

    def stopped_at_sample(self, Rover):
        """Handle switching from Stop state."""
        if self.is_event(Rover, 'can_pickup'):
            self.switch_to_state(Rover, self.state[7])  # InitiatePickup
        else:
            self.switch_to_state(Rover, self.curr_state)

    def initiating_pickup(self, Rover):
        """Handle switching from InitiatePickup state."""
        self.switch_to_state(Rover, self.state[8])  # WaitForPickupInitiate

    def waiting_pickup_initiate(self, Rover):
        """Handle switching from WaitForPickupInitiate state."""
        if Rover.picking_up == 1:
            self.switch_to_state(Rover, self.state[9])  # WaitForPickupFinish
        else:
            self.switch_to_state(Rover, self.curr_state)

    def waiting_pickup_finish(self, Rover):
        """Handle switching from WaitForPickupFinish state."""
        if Rover.picking_up == 0:
            self.switch_to_state(Rover, self.state[3])  # AvoidWall
        else:
            self.switch_to_state(Rover, self.curr_state)

    def getting_unstuck(self, Rover):
        """Handle switching from GetUnstuck state."""
        # If reached sufficient velocity or stuck while in GetUnstuck state
        # then get out of GetUnstuck state
        stucktime = 2.0  # seconds
        if Rover.vel >= 1.0 or self.is_stuck_for(Rover, stucktime):
            if Rover.going_home:
                self.switch_to_state(Rover, self.state[11])  # ReturnHome
            else:
                self.switch_to_state(Rover, self.state[1])  # FollowWall
        else:
            self.switch_to_state(Rover, self.curr_state)

    def returning_home(self, Rover):
        """Handle switching from ReturnHome state."""
        # time in seconds allowed to remain stuck in this state
        stucktime = 2.0
        if self.is_event(Rover, 'at_front_obstacle'):
            self.switch_to_state(Rover, self.state[11])  # ReturnHome
        elif self.is_event(Rover, 'reached_home'):
            self.switch_to_state(Rover, self.state[12])  # Park
        elif self.is_stuck_for(Rover, stucktime):
            self.switch_to_state(Rover, self.state[10])  # GetUnstuck
        else:
            self.switch_to_state(Rover, self.curr_state)

    def parking(self, Rover):
        """Handle switching from Park state."""
        self.switch_to_state(Rover, self.state[12])  # Remain in Park

    def execute(self, Rover):
        """Select and execute the current state action."""
        pass
