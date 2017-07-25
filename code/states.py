"""Module for rover states."""

__author__ = 'Salman Hashmi'
__license__ = 'BSD License'


import numpy as np

from perception import world_to_rover, to_polar_coords


class FindWall():
    """Create a class to represent FindWall state."""

    def __init__(self):
        """Initialize a FindWall instance."""
        self.name = 'Find Wall'

    def execute(self, Rover):
        """Execute the FindWall state action."""
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = 15


class FollowWall():
    """Create a class to represent FollowWall state."""

    def __init__(self):
        """Initialize a FollowWall instance."""
        self.throttle_setting = 0.8
        self.wall_angle_offset = 9.2
        self.name = 'Follow Wall'

    def execute(self, Rover):
        """Execute the FollowWall state action."""
        if Rover.vel < Rover.max_vel:
            Rover.throttle = self.throttle_setting
        else:
            Rover.throttle = 0

        Rover.brake = 0
        Rover.steer = np.clip(
            np.mean(Rover.nav_angles_left * 180 / np.pi),
            -15, 15) - self.wall_angle_offset


class TurnToWall():
    """Create a class to represent TurnToWall state."""

    def __init__(self):
        """Initialize a TurnToWall instance."""
        self.brake_setting = 10
        self.name = 'Turn To Wall'

    def execute(self, Rover):
        """Execute the TurnToWall state action."""
        # Stop before turning
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = self.brake_setting
            Rover.steer = 0

        elif Rover.vel <= 0.2:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = 15  # turn left towards wall


class AvoidWall():
    """Create a class to represent AvoidWall state."""

    def __init__(self):
        """Initialize a AvoidWall instance."""
        self.brake_setting = 10
        self.name = 'Avoid Wall'

    def execute(self, Rover):
        """Execute the AvoidWall state action."""
        # Stop before turning away from wall
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = self.brake_setting
            Rover.steer = 0

        elif Rover.vel <= 0.2:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = -15  # turn right away from wall


class AvoidObstacles():
    """Create a class to represent AvoidObstacles state."""

    def __init__(self):
        """Initialize a AvoidObstacles instance."""
        self.brake_setting = 10
        self.name = 'Avoid Obstacles'

    def execute(self, Rover):
        """Execute the AvoidObstacles state action."""
        # Stop before avoiding obstacles
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = self.brake_setting
            Rover.steer = 0

        elif Rover.vel <= 0.2:
            Rover.throttle = 0
            Rover.brake = 0
            # Turn left or right depending on availability
            # of nav terrain
            if np.mean(Rover.nav_angles) < -0.3:
                Rover.steer = -15
            elif np.mean(Rover.nav_angles) > 0.3:
                Rover.steer = 15
            else:  # e.g. if nav_angles are NaN
                Rover.steer = 0
                Rover.throttle = -1.0


class GoToSample():
    """Create a class to represent GoToSample state."""

    def __init__(self):
        """Initialize a GoToSample instance."""
        self.throttle_setting = 0.39
        self.approach_vel = 1.0
        self.approach_angle_bias = -0.05
        self.drive_angle_offset = -3
        self.brake_setting = 10
        self.yaw_left_setting = 15
        self.yaw_right_setting = -15
        self.name = 'Go to Sample'

    def execute(self, Rover):
        """Execute the GoToSample state action."""
        mean_rock_angle = np.mean(Rover.rock_angles) + self.approach_angle_bias
        # stop before going to sample
        if Rover.vel > self.approach_vel:
            Rover.throttle = 0
            Rover.brake = self.brake_setting
            Rover.steer = 0

        elif(Rover.vel <= self.approach_vel):
            # yaw left if sample to left more than 0.4 rads
            if mean_rock_angle >= 0.4:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = self.yaw_left_setting
            # yaw right if sample to right more than -0.4 rads
            elif mean_rock_angle <= -0.4:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = self.yaw_right_setting
            # otherwise drive in the direction of mean_rock_angle
            elif ((mean_rock_angle < 0.4 and mean_rock_angle > -0.4) or
                  math.isnan(mean_rock_angle)):
                Rover.brake = 0
                Rover.throttle = self.throttle_setting

                mean_rock_angle_deg = np.mean(
                    Rover.rock_angles * 180 / np.pi) + self.drive_angle_offset

                Rover.steer = np.clip(mean_rock_angle_deg,
                                      self.yaw_right_setting,
                                      self.yaw_left_setting)

                Rover.curr_rock_angle = mean_rock_angle_deg


class InitiatePickup():
    """Create a class to represent InitiatePickup state."""

    def __init__(self):
        """Initialize a InitiatePickup instance."""
        self.name = 'Initiate Pickup'

    def execute(self, Rover):
        """Execute the InitiatePickup state action."""
        Rover.send_pickup = True


class WaitForPickupInitiate():
    """Create a class to represent WaitForPickupInitiate state."""

    def __init__(self):
        """Initialize a WaitForPickupInitiate instance."""
        self.name = 'Wait..'

    def execute(self, Rover):
        """Execute the WaitForPickupInitiate state action."""
        pass


class WaitForPickupFinish():
    """Create a class to represent WaitForPickupFinish state."""

    def __init__(self):
        """Initialize a WaitForPickupFinish instance."""
        self.name = 'Pickup Sample.'

    def execute(self, Rover):
        """Execute the WaitForPickupFinish state action."""
        pass


class ReturnHome():
    """Create a class to represent ReturnHome state."""

    def __init__(self):
        """Initialize a ReturnHome instance."""
        self.home_pixpts_wf = np.array([99.7]), np.array([85.6])
        # home approach velocities
        self.max_vel = 2.0
        self.slow_vel = 1.0
        self.park_vel = 0.5
        # corresponding throttle settings
        self.max_throttle_set = 0.8
        self.slow_throttle_set = 0.2
        self.park_throttle_set = 0.3
        # brake setting
        self.brake_set = 10
        self.name = 'Return Home'

    def execute(self, Rover):
        """Execute the ReturnHome state action."""
        # Define conversion factor between radians and degrees
        to_deg = 180./np.pi
        # Transform home coordinates to rover frame
        home_pixpts_rf = world_to_rover(self.home_pixpts_wf, Rover.pos)
        home_distances, home_headings = to_polar_coords(home_pixpts_rf)
        # Update Rover home polar coordinates
        Rover.home_distance = np.mean(home_distances)
        Rover.home_heading = np.mean(home_headings)
        # Drive at a weighted average of home and nav headings using 3:7 ratio
        avg_nav_heading = np.mean(Rover.nav_angles)
        home_nav_heading = 0.3*Rover.home_heading + (1 - 0.3)*avg_nav_heading
        # Keep within max velocity
        if Rover.vel < self.max_vel:
            Rover.throttle = self.max_throttle_set
        else:
            Rover.throttle = 0
        # NOTE: Below distances are in meters
        # Approach at pure nav heading
        if Rover.home_distance > 450:
            Rover.brake = 0
            Rover.steer = np.clip(avg_nav_heading*to_deg, -15, 15)
        # Approach at the weighted average home plus nav headings
        elif 200 < Rover.home_distance <= 450:
            Rover.brake = 0
            Rover.steer = np.clip(home_nav_heading*to_deg, -15, 15)
        # Approach at home plus nav headings and slow down
        elif 100 < Rover.home_distance <= 200:
            if Rover.vel < self.slow_vel:
                Rover.throttle = slow_throttle_set
            else:
                Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = np.clip(home_nav_heading*to_deg, -15, 15)
        # Precisely approach at pure home heading and slow down for parking
        elif Rover.home_distance <= 100:
            if Rover.vel > self.park_vel:
                Rover.throttle = 0
                Rover.brake = self.brake_set
                Rover.steer = 0
            elif Rover.vel <= self.park_vel:
                Rover.brake = 0
                if Rover.home_heading >= 0.4:
                    Rover.throttle = 0
                    Rover.steer = 15
                elif Rover.home_heading <= -0.4:
                    Rover.throttle = 0
                    Rover.steer = -15
                elif -0.4 < Rover.home_heading < 0.4:
                    Rover.throttle = self.park_throttle_set
                    Rover.steer = np.clip(Rover.home_heading*to_deg, -15, 15)


class Stop():
    """Create a class to represent Stop state."""

    def __init__(self):
        """Initialize a Stop instance."""
        self.brake_setting = 10
        self.name = 'Stop'

    def execute(self, Rover):
        """Execute the Stop state action."""
        Rover.throttle = 0
        Rover.brake = self.brake_setting
        Rover.steer = 0


class FullStop():
    """Create a class to represent FullStop state."""

    def __init__(self):
        """Initialize a FullStop instance."""
        self.name = 'Full Stop'

    def execute(self, Rover):
        """Execute the FullStop state action."""
        pass
